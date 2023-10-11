#include <iostream>
#include "vec3.h"
#include "ray.h"
#include "color.h"

color ray_color(const ray& r) {
    // scale the ray direction to unit length -1.0 < y < 1.0
    vec3 unit_direction = unit_vector(r.direction());
    // lerp white and blue
    auto a = 0.5*(unit_direction.y() + 1.0);
    return (1.0-a)*color(1.0, 1.0, 1.0) + a*color(0.5, 0.7, 1.0);
}

int main() {

    // Image
    auto aspect_ratio = 16.0 / 9.0;
    int image_width = 400;
    int image_height = static_cast<int>(image_width / aspect_ratio);
    image_height = (image_height < 1) ? 1 : image_height; // ensure image height is at least 1

    // Camera
    auto focal_length = 1.0;
    auto viewport_height = 2.0; // arbitrary
    auto viewport_width = viewport_height * (static_cast<double>(image_width)/image_height); // <1 is ok bc real valued
    auto camera_center = point3(0, 0, 0);
    auto viewport_u = vec3(viewport_width, 0, 0); // horizontal vp edge vec
    auto viewport_v = vec3(0, -viewport_height, 0); // vertical vp edge vec
    auto pixel_delta_u = viewport_u / image_width; // space between horizontal pixel centers
    auto pixel_delta_v = viewport_v / image_height; // space between vertical pixel centers

    // Calculate the location of the upper left pixel
    auto viewport_upper_left = camera_center - vec3(0, 0, focal_length) - viewport_u/2 - viewport_v/2;
    auto pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);

    // Render
    std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";

    for (int j = 0; j < image_height; ++j) {
        std::clog << "\rScanlines remaining: " << (image_height - j) << ' ' << std::flush;
        for (int i = 0; i < image_width; ++i) {
            auto pixel_center = pixel00_loc + (i * pixel_delta_u) + (j * pixel_delta_v);
            auto ray_direction = pixel_center - camera_center;
            ray r(camera_center, ray_direction);

            color pixel_color = ray_color(r);
            write_color(std::cout, pixel_color);
        }
    }

    std::clog << "\rDone.                 \n";

}