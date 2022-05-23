// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering

#include <algorithm>            // std::min, std::max
#include <fstream>              // std::ifstream
#include <cmath>

struct vec4 {
    float x, y, z, w;
};

// Helper functions
void register_glfw_callbacks(window& app, glfw_state& app_state);

float detR(float H[16]) {
    return H[0]*(H[5]*H[10]-H[9]*H[6]) - H[4]*(H[1]*H[10]-H[2]*H[9]) + H[8]*(H[1]*H[6]-H[5]*H[2]);
}

struct vec4 multi_4x4_and_4x1(float H[16], struct vec4 vec0) {
    struct vec4 vec = {0, 0, 0, 0};
    vec.x = H[0]*vec0.x + H[4]*vec0.y + H[8]*vec0.z + H[12]*vec0.w;
    vec.y = H[1]*vec0.x + H[5]*vec0.y + H[9]*vec0.z + H[13]*vec0.w;
    vec.z = H[2]*vec0.x + H[6]*vec0.y + H[10]*vec0.z + H[14]*vec0.w;
    vec.w = H[3]*vec0.x + H[7]*vec0.y + H[11]*vec0.z + H[15]*vec0.w;
    return vec;
}

struct vec4 beta_multi_4x4_and_4x1(float H[16], struct vec4 vec0) {
    struct vec4 vec = {0, 0, 0, 0};
    vec.x = H[0]*vec0.x + H[1]*vec0.y + H[2]*vec0.z + H[3]*vec0.w;
    vec.y = H[4]*vec0.x + H[5]*vec0.y + H[6]*vec0.z + H[7]*vec0.w;
    vec.z = H[8]*vec0.x + H[9]*vec0.y + H[10]*vec0.z + H[11]*vec0.w;
    vec.w = H[12]*vec0.x + H[13]*vec0.y + H[14]*vec0.z + H[15]*vec0.w;
    return vec;
}


int map[201][201] = {{0}};

void my_draw_pointcloud_wrt_world(float width, float height, glfw_state& app_state, rs2::points& points, rs2_pose& pose, float H_t265_d400[16], std::vector<rs2_vector>& trajectory)
{
    if (!points)
        return;

    // OpenGL commands that prep screen for the pointcloud
    glLoadIdentity();
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
    glClear(GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    gluPerspective(60, width / height, 0.01f, 10.0f);


    // viewing matrix
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    // rotated from depth to world frame: z => -z, y => -y
    glTranslatef(0, 0, -0.75f - app_state.offset_y * 0.05f);
    glRotated(app_state.pitch, 1, 0, 0);
    glRotated(app_state.yaw, 0, -1, 0);
    glTranslatef(0, 0, 0.5f);

    glPointSize(10);
    glLineWidth(4.5);
    glBegin(GL_LINES);
        glColor3f(1.0, 0.0, 0.0);
        glVertex3f(0.0, 0.0, 0.0);
    	glVertex3f(0.1, 0.0, 0.0);

        glColor3f(0.0, 1.0, 0.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(0.0, 0.1, 0.0);

        glColor3f(0.0, 0.0, 1.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(0.0, 0.0, 0.1);
    glEnd();

    // draw trajectory
    glEnable(GL_DEPTH_TEST);
    glLineWidth(2.0f);
    glBegin(GL_LINE_STRIP);
    for (auto&& v : trajectory)
    {
        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(v.x, v.y, v.z);
    }
    glEnd();
    glLineWidth(0.5f);
    glColor3f(1.0f, 1.0f, 1.0f);

    // T265 pose
    GLfloat H_world_t265[16];
    quat2mat(pose.rotation, H_world_t265);
    H_world_t265[12] = pose.translation.x;
    H_world_t265[13] = pose.translation.y;
    H_world_t265[14] = pose.translation.z;

    glPointSize(10);
    glLineWidth(4.5);
    glBegin(GL_LINES);
        struct vec4 p0 = {0, 0, 0, 1};
        p0 = multi_4x4_and_4x1(H_t265_d400, p0);
        p0 = multi_4x4_and_4x1(H_world_t265, p0);
        // p0 = multi_4x4_and_4x1(H_t265_d400, p0);

        glColor3f(1.0, 0.0, 0.0);
        glVertex3f(p0.x, p0.y, p0.z);
        struct vec4 p1 = {0.1, 0, 0, 1};
        p1 = multi_4x4_and_4x1(H_t265_d400, p1);
        p1 = multi_4x4_and_4x1(H_world_t265, p1);
        // p1 = multi_4x4_and_4x1(H_t265_d400, p1);
    	glVertex3f(p1.x, p1.y, p1.z);

        glColor3f(0.0, 1.0, 0.0);
        glVertex3f(p0.x, p0.y, p0.z);
        struct vec4 p2 = {0, 0.1, 0, 1};
        p2 = multi_4x4_and_4x1(H_t265_d400, p2);
        p2 = multi_4x4_and_4x1(H_world_t265, p2);
        // p2 = multi_4x4_and_4x1(H_t265_d400, p2);
        glVertex3f(p2.x, p2.y, p2.z);

        glColor3f(0.0, 0.0, 1.0);
        glVertex3f(p0.x, p0.y, p0.z);
        struct vec4 p3 = {0, 0, sqrt(0.5), 1};
        p3 = multi_4x4_and_4x1(H_t265_d400, p3);
        p3 = multi_4x4_and_4x1(H_world_t265, p3);
        // p3 = multi_4x4_and_4x1(H_t265_d400, p3);
        glVertex3f(p3.x, p3.y, p3.z);

        glVertex3f(p0.x, p0.y, p0.z);
        struct vec4 p4 = {1.0*std::sin(PI/4.0), 0, 1.0*std::cos(PI/4.0), 1};
        p4 = multi_4x4_and_4x1(H_t265_d400, p4);
        p4 = multi_4x4_and_4x1(H_world_t265, p4);
        // p3 = multi_4x4_and_4x1(H_t265_d400, p3);
        glVertex3f(p4.x, p4.y, p4.z);

        glVertex3f(p0.x, p0.y, p0.z);
        struct vec4 p5 = {1.0*std::sin(-PI/4.0), 0, 1.0*std::cos(-PI/4.0), 1};
        p5 = multi_4x4_and_4x1(H_t265_d400, p5);
        p5 = multi_4x4_and_4x1(H_world_t265, p5);
        // p3 = multi_4x4_and_4x1(H_t265_d400, p3);
        glVertex3f(p5.x, p5.y, p5.z);
    glEnd();
    glColor3f(1.0f, 1.0f, 1.0f);

    double a1 = p4.x - p0.x;
    double a2 = p4.z - p0.z;

    double b1 = p5.x - p0.x;
    double b2 = p5.z - p0.z;

    for (int i = 0; i < 201; i++) {
        for (int j = 0; j < 201; j++) {
            double s = b2 * ((double)i/20.0 - 5.0 - p0.x) - b1 * ((double)j/20.0 - 5.0 - p0.y) ;
            s /= a1 * b2 - b1 * a2;
            double t = -a2 * ((double)i/20.0 - 5.0 - p0.x) + a1 * ((double)j/20.0 - 5.0 - p0.y);
            t /= a1 * b2 - b1 * a2;
            if (s > 0.05 && s < 0.95 && t > 0.05 && t < 0.95) {
                map[i][j] = 0;
            }
        }
    }

    auto vertices = points.get_vertices();
    for (int i = 0; i < points.size(); i += 50)
    {
        if (0.3 < vertices[i].z)
        {
            // upload the point and texture coordinates only for points we have depth data for
            struct vec4 p = {vertices[i].x, vertices[i].y, vertices[i].z, 1};
            p = multi_4x4_and_4x1(H_t265_d400, p);
            p = multi_4x4_and_4x1(H_world_t265, p);

            if (p.y > -0.3 && p.y < 0.3) {
                if (p.x >= -5.0 && p.x <= 5.0 && p.z >= -5.0 && p.z <= 5.0) {
                    map[(int)round(p.x * 20) + 100][(int)round(p.z * 20) + 100]++;
                }
            }
        }
    }

    for (int i = 0; i < 201; i++) {
        for (int j = 0; j < 201; j++) {
            if (map[i][j] > 10) {
                glBegin(GL_LINE_LOOP);
                    glLineWidth(2.0f);
                    glColor3f(0.0f, 1.0f, 1.0f);
                    glVertex3f(i/20.0 - 5 - 0.025, 0, j/20.0 - 5 - 0.025);
                    glVertex3f(i/20.0 - 5 - 0.025, 0, j/20.0 - 5 + 0.025);
                    glVertex3f(i/20.0 - 5 + 0.025, 0, j/20.0 - 5 + 0.025);
                    glVertex3f(i/20.0 - 5 + 0.025, 0, j/20.0 - 5 - 0.025);
                glEnd();
            }
        }
    }

    glMultMatrixf(H_world_t265);

    // T265 to D4xx extrinsics
    glMultMatrixf(H_t265_d400);


    glPointSize(width / 640);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, app_state.tex.get_gl_handle());
    float tex_border_color[] = { 0.8f, 0.8f, 0.8f, 0.8f };
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, tex_border_color);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, 0x812F); // GL_CLAMP_TO_EDGE
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, 0x812F); // GL_CLAMP_TO_EDGE
    glBegin(GL_POINTS);

    /* this segment actually prints the pointcloud */
    // auto vertices = points.get_vertices();              // get vertices
    auto tex_coords = points.get_texture_coordinates(); // and texture coordinates
    for (int i = 0; i < points.size(); i++)
    {
        if (0.3 < vertices[i].z)
        {
            // upload the point and texture coordinates only for points we have depth data for
            // struct vec4 p = {vertices[i].x, vertices[i].y, vertices[i].z, 1};
            // p = multi_4x4_and_4x1(H_t265_d400, p);
            // p = multi_4x4_and_4x1(H_world_t265, p);

            glVertex3fv(vertices[i]);
            glColor3f(1.0f, 1.0f, 1.0f);
            // glPointSize(width / 640);
            // glVertex3f(p.x, p.y, p.z);
            glTexCoord2fv(tex_coords[i]);
        }
    }

    // OpenGL cleanup
    glEnd();

    // glPointSize(10);
    // glLineWidth(4.5);
    // glBegin(GL_LINES);
    //     glColor3f(1.0, 0.0, 0.0);
    //     glVertex3f(0.0, 0.0, 0.0);
    // 	glVertex3f(0.1, 0.0, 0.0);
    //
    //     glColor3f(0.0, 1.0, 0.0);
    //     glVertex3f(0.0, 0.0, 0.0);
    //     glVertex3f(0.0, 0.1, 0.0);
    //
    //     glColor3f(0.0, 0.0, 1.0);
    //     glVertex3f(0.0, 0.0, 0.0);
    //     glVertex3f(0.0, 0.0, 0.1);
    // glEnd();

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopAttrib();
}

int main(int argc, char * argv[]) try
{
    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense Tracking and Depth Example");
    // Construct an object to manage view state
    glfw_state app_state;
    // register callbacks to allow manipulation of the pointcloud
    register_glfw_callbacks(app, app_state);

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;
    // store pose and timestamp
    rs2::pose_frame pose_frame(nullptr);
    std::vector<rs2_vector> trajectory;

    rs2::context                          ctx;        // Create librealsense context for managing devices
    std::map<std::string, rs2::colorizer> colorizers; // Declare map from device serial number to colorizer (utility class to convert depth data RGB colorspace)
    std::vector<rs2::pipeline>            pipelines;

    // Capture serial numbers before opening streaming
    std::vector<std::string>              serials;
    for (auto&& dev : ctx.query_devices())
        serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));


    // Start a streaming pipe per each connected device
    for (auto&& serial : serials)
    {
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        cfg.enable_device(serial);
        pipe.start(cfg);
        pipelines.emplace_back(pipe);
        // Map from each device's serial number to a different colorizer
        colorizers[serial] = rs2::colorizer();
    }

    // extrinsics
    // depth w.r.t. tracking (column-major)
    float H_t265_d400[16] =  {1, 0, 0, 0,
                              0,-1, 0, 0,
                              0, 0,-1, 0,
                              0, 0, 0, 1};
    std::string fn = "./H_t265_d400.cfg";
    std::ifstream ifs(fn);
    if (!ifs.is_open()) {
        std::cerr << "Couldn't open " << fn << std::endl;
        return -1;
    }
    else {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                ifs >> H_t265_d400[i+4*j];  // row-major to column-major
            }
        }
    }
    float det = detR(H_t265_d400);
    if (fabs(1-det) > 1e-6) {
        std::cerr << "Invalid homogeneous transformation matrix input (det != 1)" << std::endl;
        return -1;
    }

    while (app) // Application still alive?
    {
        for (auto &&pipe : pipelines) // loop over pipelines
        {
            // Wait for the next set of frames from the camera
            auto frames = pipe.wait_for_frames();


            auto color = frames.get_color_frame();

            // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
            if (!color)
                color = frames.get_infrared_frame();

            // Tell pointcloud object to map to this color frame
            if (color)
                pc.map_to(color);

            auto depth = frames.get_depth_frame();

            // Generate the pointcloud and texture mappings
            if (depth)
                points = pc.calculate(depth);

            // Upload the color frame to OpenGL
            if (color)
                app_state.tex.upload(color);


            // pose
            auto pose = frames.get_pose_frame();
            if (pose) {
                pose_frame = pose;

                // Print the x, y, z values of the translation, relative to initial position
                auto pose_data = pose.get_pose_data();
                std::cout << "\r" << "Device Position: " << std::setprecision(3) << std::fixed << pose_data.translation.x << " " << pose_data.translation.y << " " << pose_data.translation.z << " (meters)";

                // add new point in the trajectory (if motion large enough to reduce size of traj. vector)
                if (trajectory.size() == 0)
                    trajectory.push_back(pose_data.translation);
                else {
                    rs2_vector prev = trajectory.back();
                    rs2_vector curr = pose_data.translation;
                    if (sqrt(pow((curr.x - prev.x), 2) + pow((curr.y - prev.y), 2) + pow((curr.z - prev.z), 2)) > 0.002)
                        trajectory.push_back(pose_data.translation);
                }
             }
        }

        // Draw the pointcloud
        if (points && pose_frame) {
            rs2_pose pose =  pose_frame.get_pose_data();
            my_draw_pointcloud_wrt_world(app.width(), app.height(), app_state, points, pose, H_t265_d400, trajectory);
        }
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
