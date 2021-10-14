#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};// binary data and ate stands for at the end:means seek to the end of the input Stream.
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();// with line 15 of parameter ate, we can use .tellg() function to determine the size of the input stream.
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);// we can seek back to the beginning of the input stream of line 22.
    is.read((char*)contents.data(), size);// read all of the input stream into the contents vector.

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);// we will return the contents vector using the standard move of contents.
    // move allows you to transfer the contents of this vector to another variable without using pointers or references.
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )// parse the command line arguments of dash f. which allows you to specify the osm datafile that you want to use.
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";// if you don't specify an osm datafile, then osm datafile will be set to map.osm
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);// from line 46 to 52, use ReadFile function to read the contents of this osm datafile into the vector osmdata
    }
    
    // TODO 1: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
    // user input for these values using std::cin. Pass the user input to the
    // RoutePlanner object below in place of 10, 10, 90, 90.
    
    float start_x=-1, start_y=-1, end_x=-1, end_y=-1;

    while(!(std::cin)||(start_x<0)||(start_x>100)){
        std::cout<<"Input start_x value must be in range [0, 100]"<<std::endl;
        std::cin>>start_x;
        std::cin.clear();
    }
    while(!(std::cin)||(start_y<0)||(start_y>100)){
        std::cout<<"Input start_y value must be in range [0, 100]"<<std::endl;
        std::cin>>start_y;
        std::cin.clear();
    }
    while(!(std::cin)||(end_x<0)||(end_x>100)){
        std::cout<<"Input end_x value must be in range [0, 100]"<<std::endl;
        std::cin>>end_x;
        std::cin.clear();
    }
    while(!(std::cin)||(end_y<0)||(end_y>100)){
        std::cout<<"Input end_y value must be in range [0, 100]"<<std::endl;
        std::cin>>end_y;
        std::cin.clear();
    }

    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    //RoutePlanner route_planner{model, 10, 10, 90, 90};
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
