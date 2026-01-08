#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>

// TrackingObject was message of sensor fusion, please refer AJOU-NICE Notion(secret). 
// #include <tracking_msg/TrackingObject.h> 
// #include <tracking_msg/TrackingObjectArray.h>

#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <cmath>  // for std::hypot, std::ceil

class VelodyneToGrid {
public:
    VelodyneToGrid() {
        sub_1 = nh_.subscribe("/velodyne_points", 1, &VelodyneToGrid::pointCloudCallback, this);
        // sub_2 = nh_.subscribe("/fusion_box/fused_3d_box", 1, &VelodyneToGrid::bBoxCallback, this);
        pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/local_map", 1);

        // OccupancyGrid 기본 설정
        grid_.info.resolution = 0.1; // 10cm
        grid_.info.width = 100;      // 10m x 10m
        grid_.info.height = 100;
        grid_.info.origin.position.x = -5.0;
        grid_.info.origin.position.y = -5.0;
        grid_.info.origin.position.z = -0.7;
        grid_.data.resize(grid_.info.width * grid_.info.height, 0);
    }

    // void bBoxCallback(const tracking_msg::TrackingObjectArray::ConstPtr& msg){
    //     if (msg->size < 1){
    //         ROS_INFO("no tracking objects..");
    //     }
    //     else{
    //         objects.clear();
    //         for (const auto& obj : msg->array){
    //             const auto& P = obj.bbox.points;
    //             static const int pick[8] = {0,1,2,3,10,12,14,16};
    //             const geometry_msgs::Point c1 = P[pick[0]];
    //             const geometry_msgs::Point c2 = P[pick[1]];
    //             const geometry_msgs::Point c3 = P[pick[2]];
    //             const geometry_msgs::Point c4 = P[pick[3]];
    //             objects.push_back({{c1.x,c1.y}, {c2.x,c2.y}, {c3.x,c3.y}, {c4.x,c4.y}});
    //             ROS_INFO("%.2f ,%.2f",c1.x,c1.y);
    //             ROS_INFO("%.2f ,%.2f",c2.x,c2.y);
    //             ROS_INFO("%.2f ,%.2f",c3.x,c3.y);
    //             ROS_INFO("%.2f ,%.2f",c4.x,c4.y);
    //         }
    //     }
    // }

    // std::pair<double,double> worldToGrid(double x, double y, double origin_x, double origin_y, double res){
    //     double gx = (x - origin_x) / res;
    //     double gy = (y - origin_y) / res;
    //     return {gx, gy};
    // }

    // Scanline polygon rasterization
    std::vector<std::vector<bool>> rasterizePolygonScanline(
            const std::vector<std::pair<double,double>>& verts,
            int width, int height)
    {
        std::vector<std::vector<bool>> filled(height, std::vector<bool>(width, false));
        int n = verts.size();

        // edges
        std::vector<std::pair<std::pair<double,double>, std::pair<double,double>>> edges;
        for(int i=0; i<n; i++)
        {
            edges.push_back({verts[i], verts[(i+1)%n]});
        }

        for(int j=0; j<height; j++)
        {
            double y_scan = j + 0.5;
            std::vector<double> xs;

            for(auto &e : edges)
            {
                double x1 = e.first.first, y1 = e.first.second;
                double x2 = e.second.first, y2 = e.second.second;

                if(y1 == y2) continue;

                double ymin = std::min(y1,y2);
                double ymax = std::max(y1,y2);
                if(!(ymin <= y_scan && y_scan < ymax)) continue;

                double t = (y_scan - y1) / (y2 - y1);
                double x_int = x1 + t*(x2 - x1);
                xs.push_back(x_int);
            }

            std::sort(xs.begin(), xs.end());

            for(size_t k=0; k+1<xs.size(); k+=2)
            {
                double x_start = xs[k];
                double x_end   = xs[k+1];

                int i_min = (int)std::ceil(x_start - 0.5);
                int i_max = (int)std::floor(x_end   - 0.5);

                if(i_min < 0) i_min = 0;
                if(i_max >= width) i_max = width-1;

                for(int i=i_min; i<=i_max; i++)
                {
                    filled[j][i] = true;
                }
            }
        }

        return filled;
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // OccupancyGrid 초기화
        std::fill(grid_.data.begin(), grid_.data.end(), 0);

        int height = grid_.info.height;
        int width = grid_.info.width;

        // for(auto &obj_ : objects){
        //     // 월드→grid 좌표
        //     std::vector<std::pair<double,double>> verts_grid;
        //     for(auto &p : obj_)
        //     {
        //         verts_grid.push_back(worldToGrid(p.first, p.second,-5, -5, 0.1)); // origin, resolution
        //         std::pair<double, double> grid_coord = worldToGrid(p.first, p.second,-5, -5, 0.1);
        //         ROS_INFO("verts push back %f, %f",grid_coord.first, grid_coord.second); // origin, resolution
        //     }

        //     auto filled = rasterizePolygonScanline(verts_grid, width, height);

        //     for(int y=0; y < height; y++)
        //     {
        //         for(int x=0; x < width; x++)
        //         {
        //             if(filled[y][x])
        //             {
        //                 grid_.data[y * width + x] = 100;
        //             }
        //         }
        //     }
        // }

        // ------------------------------------------------ //

        double z_threshold = -0.4;  // 바닥 필터링 기준

        for (const auto& pt : cloud.points) {
            if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) continue;
            if (pt.z < z_threshold) continue;  // 바닥 제거

            int x_idx = static_cast<int>((pt.x - grid_.info.origin.position.x) / grid_.info.resolution);
            int y_idx = static_cast<int>((pt.y - grid_.info.origin.position.y) / grid_.info.resolution);

            if (x_idx >= 0 && x_idx < grid_.info.width &&
                y_idx >= 0 && y_idx < grid_.info.height) {
                int index = y_idx * grid_.info.width + x_idx;
                grid_.data[index] = 100;
            }
        }

        for (int x = 30; x <= 50; x++) {
            for (int y = 43; y <= 55; y++) {
                int index = y * grid_.info.width + x;
                grid_.data[index] = 0;
            }   
        }


        // --------------- obstacle inflation -----------------------//
        std::vector<int8_t> original = grid_.data;  // 원본 복사
        double dilation_radius = 0.4;               // 확장 반경 (예: 30cm)
        int dilation_cells = std::ceil(dilation_radius / grid_.info.resolution);
        

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int idx = y * width + x;

                if (original[idx] == 100) {
                    for (int dy = -dilation_cells; dy <= dilation_cells; ++dy) {
                        for (int dx = -dilation_cells; dx <= dilation_cells; ++dx) {
                            int nx = x + dx;
                            int ny = y + dy;

                            if (nx < 0 || ny < 0 || nx >= width || ny >= height)
                                continue;

                            if (std::hypot(dx, dy) * grid_.info.resolution <= dilation_radius) {
                                int nidx = ny * width + nx;
                                if (grid_.data[nidx] != 100)
                                    grid_.data[nidx] = 99;  // 확장된 장애물 셀
                            }
                        }
                    }
                }
            }
        }
        // ------------------------------------------------------

        // 차량 자체를 비우기 (장애물 오인 방지)
        for (int x = 30; x <= 50; x++) {
            for (int y = 43; y <= 55; y++) {
                int index = y * grid_.info.width + x;
                grid_.data[index] = 0;
            }
        }

        grid_.header.stamp = ros::Time::now();
        grid_.header.frame_id = "velodyne";
        pub_.publish(grid_);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_1;
    ros::Subscriber sub_2;
    ros::Publisher pub_;
    nav_msgs::OccupancyGrid grid_;
    std::vector<std::vector<std::pair<double,double>>> objects;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "velodyne_to_grid");
    VelodyneToGrid vtg;
    ros::spin();
    return 0;
}
