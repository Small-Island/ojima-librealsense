// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering

#include <algorithm>            // std::min, std::max

#include <arpa/inet.h>
#include <unistd.h>

int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
struct sockaddr_in addr;

struct My_udp_data {
    double obstacle_rate = 0.0;
};

// Helper functions
void register_glfw_callbacks(window& app, glfw_state& app_state);

void my_draw_pointcloud(float width, float height, glfw_state& app_state, rs2::points& points)
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

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

    glTranslatef(0, 0, +0.5f + app_state.offset_y * 0.05f);
    glRotated(app_state.pitch, 1, 0, 0);
    glRotated(app_state.yaw, 0, 1, 0);
    glTranslatef(0, 0, -0.5f);

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
    auto vertices = points.get_vertices();              // get vertices
    auto tex_coords = points.get_texture_coordinates(); // and texture coordinates
    int sum = 0;
    for (int i = 0; i < points.size(); i++)
    {
        // if (0 < vertices[i].z && vertices[i].z < 1.0 && vertices[i].y < 0)
        // {
        //     // upload the point and texture coordinates only for points we have depth data for
        //     glVertex3fv(vertices[i]);
        //     glTexCoord2fv(tex_coords[i]);
        // }
        if (0 < vertices[i].z && -0.35 < vertices[i].x && vertices[i].x < 0.35 && vertices[i].y < 0.55 && vertices[i].z < 1.0)
        {
            if (vertices[i].z < 1.0) {
                // upload the point and texture coordinates only for points we have depth data for
                glVertex3fv(vertices[i]);
                glTexCoord2fv(tex_coords[i]);
                sum++;
            }
        }
    }

    printf("sum: %d\n", sum);

    struct My_udp_data my_udp_data;

    if (sum > 500) {
        my_udp_data.obstacle_rate = 1;
    }
    else {
        my_udp_data.obstacle_rate = 0;
    }
    sendto(sockfd, &my_udp_data, sizeof(struct My_udp_data), 0, (struct sockaddr *)&addr, sizeof(addr));

    // OpenGL cleanup
    glEnd();
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopAttrib();
}

int main(int argc, char * argv[]) try
{
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    addr.sin_port = htons(4001);

    // Create a simple OpenGL window for rendering:
    // window app(1280, 720, "RealSense Pointcloud Example");
    // Construct an object to manage view state
    // glfw_state app_state;
    // register callbacks to allow manipulation of the pointcloud
    // register_glfw_callbacks(app, app_state);

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();

    // while (app) // Application still alive?
    while (1)
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();

        // auto color = frames.get_color_frame();

        // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
        // if (!color)
        //     color = frames.get_infrared_frame();

        // Tell pointcloud object to map to this color frame
        // pc.map_to(color);

        auto depth = frames.get_depth_frame();

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);
        auto vertices = points.get_vertices();
        auto tex_coords = points.get_texture_coordinates();

        int sum = 0;
        for (int i = 0; i < points.size(); i = i + 100)
        {
            // if (0 < vertices[i].z && vertices[i].z < 1.0 && vertices[i].y < 0)
            // {
            //     // upload the point and texture coordinates only for points we have depth data for
            //     glVertex3fv(vertices[i]);
            //     glTexCoord2fv(tex_coords[i]);
            // }
            if (0 < vertices[i].z && -0.35 < vertices[i].x && vertices[i].x < 0.35 && vertices[i].y < 0.55 && vertices[i].z < 1.0)
            {
                if (vertices[i].z < 1.0) {
                    // upload the point and texture coordinates only for points we have depth data for
                    // glVertex3fv(vertices[i]);
                    // glTexCoord2fv(tex_coords[i]);
                    sum++;
                }
            }
        }

        printf("sum: %d\n", sum);

        struct My_udp_data my_udp_data;

        if (sum > 500) {
            my_udp_data.obstacle_rate = 1;
        }
        else {
            my_udp_data.obstacle_rate = 0;
        }
        sendto(sockfd, &my_udp_data, sizeof(struct My_udp_data), 0, (struct sockaddr *)&addr, sizeof(addr));


        // Upload the color frame to OpenGL
        // app_state.tex.upload(color);

        // Draw the pointcloud
        // my_draw_pointcloud(app.width(), app.height(), app_state, points);
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
