#include <fstream>

#include "viewer/Renderer.h"
#include "tracker/Tracker.h"
#include "CloudManager.h"

int main(int argc, char *argv[])
{
    int64_t freq = 100;            // Frequency of the thread dedicated to process the point cloud
    std::string log_path = "log";  // TODO: define the path to the log folder

    std::ifstream dataFile(log_path, std::ios::in | std::ios::binary);
    if (!dataFile)
    {
        std::cerr << "ERROR: The file '" << log_path << "' does not exist. Exiting.\n";
        return 1;
    }

    // Init renderer
    viewer::Renderer renderer;
    renderer.initCamera(viewer::CameraAngle::XY);
    renderer.clearViewer();

    // Instantiate the tracker
    Tracker tracker;
    Tracker::Area area;
    area.min_x = -2;
    area.max_x = 2;
    area.min_y = -2;
    area.max_y = 2;

    viewer::Box box_area;

    box_area.x_min = area.min_x;
    box_area.x_max = area.max_x;
    box_area.y_min = area.min_y;
    box_area.y_max = area.max_y;
    box_area.z_min = -1;
    box_area.z_max = -1;

    tracker.setArea(area);

    // Spawn the thread that process the point cloud and performs the clustering
    CloudManager lidar_cloud(log_path, freq, renderer);
    std::thread t(&CloudManager::startCloudManager, &lidar_cloud);

    std::stringstream longest_path_display;
    

    while (true)
    {
        // Clear the render
        renderer.clearViewer();

        while (!lidar_cloud.new_measurement)
            ; // wait for new data (we will execute the following code each 100ms)

        // fetch data
        lidar_cloud.mtxData.lock();
        auto cloud = lidar_cloud.getCloud();
        auto color = lidar_cloud.getColor();
        auto boxes = lidar_cloud.getBoxes();
        auto centroids_x = lidar_cloud.getCentroidsX();
        auto centroids_y = lidar_cloud.getCentroidsY();
        lidar_cloud.new_measurement = false;
        lidar_cloud.mtxData.unlock();

        // render pointcloud and boxes
        renderer.renderPointCloud(cloud, "pointCloud", color);
        for (size_t i = 0; i < boxes.size(); ++i)
            renderer.renderBox(boxes[i], i);

        // Call the tracker on the detected clusters
        tracker.track(centroids_x, centroids_y, renderer.getLidarStatus());

        // retrieve tracklets and render the trackers
        auto tracks = tracker.getTracks();
        for (size_t i = 0; i < tracks.size(); ++i)
        {
            renderer.addCircle(tracks[i].getX(), tracks[i].getY(), tracks[i].getId());
            renderer.addText(tracks[i].getX() + 0.01, tracks[i].getY() + 0.01, tracks[i].getId());
        }

        // longest traveled path features
        auto longest_path = tracker.getLongestPath();
        longest_path_display.str(std::string());
        longest_path_display << "Longest path: ID: " << longest_path.first << " distance: " << longest_path.second;
        std::cout << longest_path_display.str() << std::endl;

        // tracklets in area feature
        renderer.renderBox(box_area, -1, {1,1,0}, 0.20);
        tracker.calcTrackletsInArea();
        std::cout << "Number of traclets in yellow area:" << tracker.getNumTrackletsInArea() << std::endl;
        std::cout << "Tracklet for most time in area:" << tracker.getLongestInAreaId() << std::endl;

        // cleanings
        std::cout << std::endl;
        renderer.spinViewerOnce();
    }

    t.join();
    return 0;
}
