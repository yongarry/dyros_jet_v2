#include "jet_controller/state_manager.h"

StateManager::StateManager(DataContainer &dc_global) : dc_global_(dc_global), rd_global_(dc_global.rd_)
{
    string urdf_path;
    ros::param::get("/jet_controller/urdf", urdf_path);
    
}