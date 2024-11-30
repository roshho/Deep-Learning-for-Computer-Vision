/*
-- Running cmake --
```
mkdir build
cd build
cmake ..
make
./realsense_pcl_capture
```
*/

#include <librealsense2/rs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

int main(int argc, char * argv[]) try
{
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    
    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    rs2::points points;

    // Start streaming with default recommended configuration
    pipe.start();

    int frame_counter = 0;
    const int frames_to_capture = 10; // Will capture 10 frames then exit

    while (frame_counter < frames_to_capture) 
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();

        auto color = frames.get_color_frame();
        if (!color)
            color = frames.get_infrared_frame();

        auto depth = frames.get_depth_frame();

        // Tell pointcloud object to map to this color frame
        pc.map_to(color);

        // Generate the pointcloud
        points = pc.calculate(depth);

        // Create PCL cloud object
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        // Get vertices
        auto vertices = points.get_vertices();
        auto tex_coords = points.get_texture_coordinates();
        
        // Fill cloud
        cloud->width = depth.get_width();
        cloud->height = depth.get_height();
        cloud->is_dense = false;
        cloud->points.resize(points.size());

        for (int i = 0; i < points.size(); i++) 
        {
            auto& p = cloud->points[i];
            p.x = vertices[i].x;
            p.y = vertices[i].y;
            p.z = vertices[i].z;

            // Get color
            if (tex_coords[i].u >= 0.0f && tex_coords[i].u <= 1.0f &&
                tex_coords[i].v >= 0.0f && tex_coords[i].v <= 1.0f) 
            {
                int x = std::min(std::max(int(tex_coords[i].u * color.get_width()), 0), 
                               int(color.get_width() - 1));
                int y = std::min(std::max(int(tex_coords[i].v * color.get_height()), 0), 
                               int(color.get_height() - 1));
                int idx = x * 3 + y * color.get_stride_in_bytes();
                const auto texture = reinterpret_cast<const uint8_t*>(color.get_data());
                
                p.r = texture[idx];
                p.g = texture[idx + 1];
                p.b = texture[idx + 2];
            }
        }

        // Save current cloud
        std::string filename = "pointcloud_" + std::to_string(frame_counter) + ".pcd";
        pcl::io::savePCDFileBinary(filename, *cloud);
        std::cout << "Saved " << filename << std::endl;

        frame_counter++;
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