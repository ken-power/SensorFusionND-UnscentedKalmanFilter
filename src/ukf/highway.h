// Handle logic for creating traffic on highway and animating it

#include "../render/render.h"
#include "../sensors/lidar.h"
#include "tools.h"

class Highway
{
public:

    explicit Highway(pcl::visualization::PCLVisualizer::Ptr & viewer)
    {

        tools_ = Tools();

        ego_car_ = Car(Vect3(0, 0, 0),
                       Vect3(4, 2, 2),
                       Color(0, 1, 0),
                       0,
                       0,
                       2,
                       "egoCar");

        Car car1(Vect3(-10, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), 5, 0, 2, "car1");

        std::vector<Accuation> car1_instructions;
        Accuation a = Accuation(0.5 * 1e6, 0.5, 0.0);
        car1_instructions.push_back(a);
        a = Accuation(2.2 * 1e6, 0.0, -0.2);
        car1_instructions.push_back(a);
        a = Accuation(3.3 * 1e6, 0.0, 0.2);
        car1_instructions.push_back(a);
        a = Accuation(4.4 * 1e6, -2.0, 0.0);
        car1_instructions.push_back(a);

        car1.setInstructions(car1_instructions);
        if(tracked_cars_[0])
        {
            UKF ukf1;
            car1.setUKF(ukf1);
        }
        traffic_.push_back(car1);

        Car car2(Vect3(25, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), -6, 0, 2, "car2");
        std::vector<Accuation> car2_instructions;
        a = Accuation(4.0 * 1e6, 3.0, 0.0);
        car2_instructions.push_back(a);
        a = Accuation(8.0 * 1e6, 0.0, 0.0);
        car2_instructions.push_back(a);
        car2.setInstructions(car2_instructions);
        if(tracked_cars_[1])
        {
            UKF ukf2;
            car2.setUKF(ukf2);
        }
        traffic_.push_back(car2);

        Car car3(Vect3(-12, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), 1, 0, 2, "car3");
        std::vector<Accuation> car3_instructions;
        a = Accuation(0.5 * 1e6, 2.0, 1.0);
        car3_instructions.push_back(a);
        a = Accuation(1.0 * 1e6, 2.5, 0.0);
        car3_instructions.push_back(a);
        a = Accuation(3.2 * 1e6, 0.0, -1.0);
        car3_instructions.push_back(a);
        a = Accuation(3.3 * 1e6, 2.0, 0.0);
        car3_instructions.push_back(a);
        a = Accuation(4.5 * 1e6, 0.0, 0.0);
        car3_instructions.push_back(a);
        a = Accuation(5.5 * 1e6, -2.0, 0.0);
        car3_instructions.push_back(a);
        a = Accuation(7.5 * 1e6, 0.0, 0.0);
        car3_instructions.push_back(a);
        car3.setInstructions(car3_instructions);
        if(tracked_cars_[2])
        {
            UKF ukf3;
            car3.setUKF(ukf3);
        }
        traffic_.push_back(car3);

        lidar_ = new Lidar(traffic_, 0);

        // render environment
        RenderHighway(0, viewer);
        ego_car_.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    void StepHighway(double ego_velocity,
                     long long timestamp,
                     int frame_per_sec,
                     pcl::visualization::PCLVisualizer::Ptr & viewer)
    {

        if(visualize_pcd_)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr trafficCloud = tools_.loadPcd(
                    "../src/sensors/data/pcd/highway_" + std::to_string(timestamp) + ".pcd");
            RenderPointCloud(viewer,
                             trafficCloud,
                             "trafficCloud",
                             Color((float) 184 / 256, (float) 223 / 256, (float) 252 / 256));
        }


        // render highway environment with poles
        RenderHighway(ego_velocity * timestamp / 1e6, viewer);
        ego_car_.render(viewer);

        for(int i = 0; i < traffic_.size(); i++)
        {
            traffic_[i].move((double) 1 / frame_per_sec, timestamp);
            if(!visualize_pcd_)
                traffic_[i].render(viewer);
            // Sense surrounding cars with lidar and radar
            if(tracked_cars_[i])
            {
                VectorXd gt(4);
                gt << traffic_[i].position.x, traffic_[i].position.y, traffic_[i].velocity * cos(traffic_[i].angle),
                        traffic_[i].velocity * sin(traffic_[i].angle);
                tools_.ground_truth.push_back(gt);
                tools_.lidarSense(traffic_[i], viewer, timestamp, visualize_lidar_);
                tools_.radarSense(traffic_[i], ego_car_, viewer, timestamp, visualize_radar_);
                tools_.ukfResults(traffic_[i], viewer, projected_time_, projected_steps_);
                VectorXd estimate(4);
                double v = traffic_[i].ukf.x_(2);
                double yaw = traffic_[i].ukf.x_(3);
                double v1 = cos(yaw) * v;
                double v2 = sin(yaw) * v;
                estimate << traffic_[i].ukf.x_[0], traffic_[i].ukf.x_[1], v1, v2;
                tools_.estimations.push_back(estimate);

            }
        }
        viewer->addText("Accuracy - RMSE:", 30, 300, 20, 1, 1, 1, "rmse");
        VectorXd rmse = tools_.CalculateRMSE(tools_.estimations, tools_.ground_truth);

        cout << "RMSE: " << rmse << endl;

        viewer->addText(" X: " + std::to_string(rmse[0]), 30, 275, 20, 1, 1, 1, "rmse_x");
        viewer->addText(" Y: " + std::to_string(rmse[1]), 30, 250, 20, 1, 1, 1, "rmse_y");
        viewer->addText("Vx: " + std::to_string(rmse[2]), 30, 225, 20, 1, 1, 1, "rmse_vx");
        viewer->addText("Vy: " + std::to_string(rmse[3]), 30, 200, 20, 1, 1, 1, "rmse_vy");

        cout << "Running stats: " << endl;
        cout << "X = " << rmse[0] << "\tY = " << rmse[1] << "\tVx = " << rmse[2] << "\tVy = " << rmse[3] << endl;

        WriteToFile(rmse);

        if(timestamp > 1.0e6)
        {
            if(rmse[0] > rmse_threshold_values_[0])
            {
                rmse_failLog_[0] = rmse[0];
                pass_ = false;
            }
            if(rmse[1] > rmse_threshold_values_[1])
            {
                rmse_failLog_[1] = rmse[1];
                pass_ = false;
            }
            if(rmse[2] > rmse_threshold_values_[2])
            {
                rmse_failLog_[2] = rmse[2];
                pass_ = false;
            }
            if(rmse[3] > rmse_threshold_values_[3])
            {
                rmse_failLog_[3] = rmse[3];
                pass_ = false;
            }
        }
        if(!pass_)
        {
            cerr << "RMSE Failed Threshold" << endl;

            viewer->addText("RMSE Failed Threshold", 30, 150, 20, 1, 0, 0, "rmse_fail");
            if(rmse_failLog_[0] > 0)
                viewer->addText(" X: " + std::to_string(rmse_failLog_[0]), 30, 125, 20, 1, 0, 0, "rmse_fail_x");
            if(rmse_failLog_[1] > 0)
                viewer->addText(" Y: " + std::to_string(rmse_failLog_[1]), 30, 100, 20, 1, 0, 0, "rmse_fail_y");
            if(rmse_failLog_[2] > 0)
                viewer->addText("Vx: " + std::to_string(rmse_failLog_[2]), 30, 75, 20, 1, 0, 0, "rmse_fail_vx");
            if(rmse_failLog_[3] > 0)
                viewer->addText("Vy: " + std::to_string(rmse_failLog_[3]), 30, 50, 20, 1, 0, 0, "rmse_fail_vy");
        }
    }

private:
    std::vector<Car> traffic_;
    Car ego_car_;
    Tools tools_;
    bool pass_ = true;
    std::vector<double> rmse_threshold_values_ = {0.30, 0.16, 0.95, 0.70};
    std::vector<double> rmse_failLog_ = {0.0, 0.0, 0.0, 0.0};
    Lidar *lidar_;

    // Parameters
    // --------------------------------
    // Set which cars to track with UKF
    std::vector<bool> tracked_cars_ = {true, true, true};

    // Visualize sensor measurements
    bool visualize_lidar_ = true;
    bool visualize_radar_ = true;
    bool visualize_pcd_ = false;

    // Predict path in the future using UKF
    double projected_time_ = 0;
    int projected_steps_ = 0;

    // --------------------------------


    void WriteToFile(VectorXd rmse_data)
    {
        const string filename = "../../results/ukf_performance_results.csv";
        const string separator = ", ";
        ofstream results_file;
        results_file.open(filename, ios_base::app);
        results_file << rmse_data[0] << separator << rmse_data[1] << separator << rmse_data[2] << separator << rmse_data[3] << endl;
    }
};
