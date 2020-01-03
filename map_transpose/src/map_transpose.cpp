//
// Created by zzh on 2019/12/30.
//
#include <map_transpose.h>
#include <iostream>
#include <string>
#include <map-manager/map-manager.h>
#include <vi-map/vi-map.h>
#include <console-common/console.h>
// Your new plugin needs to derive from ConsolePluginBase.
// (Alternatively, you can derive from ConsolePluginBaseWithPlotter if you need
// RViz plotting abilities for your VI map.)
DEFINE_string(frame_save_folder, "", "path to save frame");
DEFINE_string(mappoint_save_folder, "", "");
class map_transpose : public common::ConsolePluginBase {
public:
    // Every plugin needs to implement a getPluginId function which returns a
    // string that gives each plugin a unique name.
    std::string getPluginId() const override {
        return "map_transpose";
    }

    map_transpose(common::Console* console)
            : common::ConsolePluginBase(console) {
        // You can add your commands in here.
        addCommand(
                {"save_frame"},  // Map "hello_world" and "hello" to this
                // command.
                [this]() -> int {  // Function to call when this command is entered.
                    // This function can do anything you want. Check the other plugins
                    // under ~/maplab_ws/src/maplab/console-plugins for more examples.

                    // Here, we just print a message to the terminal.
                    std::string select_map_key;
                    if(!getSelectedMapKeyIfSet(&select_map_key)){
                        return common::kStupidUserError;
                    }
                    vi_map::VIMapManager map_manager;
                    vi_map::VIMapManager::MapReadAccess map =
                            map_manager.getMapReadAccess(select_map_key);
                    if (FLAGS_frame_save_folder.empty()){
                        std::cerr << "Please input path" << std::endl;
                        return common::kStupidUserError;
                    }
                    std::string path = FLAGS_frame_save_folder;
                    save_keyframe(*map, path);
                    std::cout << path << std::endl;
                    /*vi_map::MissionIdList mission_ids;
                    map->getAllMissionIds(&mission_ids);
                    vi_map::MissionId& id_of_first_mission = mission_ids.front();
                    pose_graph::VertexIdList vertexids_firstmisson;
                    map->getAllVertexIdsInMissionAlongGraph(id_of_first_mission, &vertexids_firstmisson);
                    for (int i=0; i < vertexids_firstmisson.size(); i++){
                        pose_graph::VertexId& vertexid = vertexids_firstmisson.at(i);
                        const vi_map::Vertex& cur_vertex = map->getVertex(vertexid);

                    }*/

                    // Every console command returns an integer, you can take one from
                    // the CommandStatus enum. kSuccess returns everything is fine.
                    // Other commonly used return values are common::kUnknownError and
                    // common::kStupidUserError.
                    return common::kSuccess;
                },
                // This is the description of your command. This will get printed when
                // you run `help` in the console.
                "This plugin is used to change map format",

                // This specifies the execution method of your command. For most
                // commands, it is sufficient to run them in sync with
                // common::Processing::Sync.
                common::Processing::Sync);
        addCommand(
                {"save_mappoint"},  // Map "hello_world" and "hello" to this
                // command.
                [this]() -> int {  // Function to call when this command is entered.
                    // This function can do anything you want. Check the other plugins
                    // under ~/maplab_ws/src/maplab/console-plugins for more examples.

                    // Here, we just print a message to the terminal.
                    std::string select_map_key;
                    if(!getSelectedMapKeyIfSet(&select_map_key)){
                        return common::kStupidUserError;
                    }
                    vi_map::VIMapManager map_manager;
                    vi_map::VIMapManager::MapReadAccess map =
                            map_manager.getMapReadAccess(select_map_key);
                    if (FLAGS_mappoint_save_folder.empty()){
                        std::cerr << "Please input path" << std::endl;
                        return common::kStupidUserError;
                    }
                    std::string path = FLAGS_mappoint_save_folder;
                    save_mappoint(*map, path);
                    std::cout << path << std::endl;
                    /*vi_map::MissionIdList mission_ids;
                    map->getAllMissionIds(&mission_ids);
                    vi_map::MissionId& id_of_first_mission = mission_ids.front();
                    pose_graph::VertexIdList vertexids_firstmisson;
                    map->getAllVertexIdsInMissionAlongGraph(id_of_first_mission, &vertexids_firstmisson);
                    for (int i=0; i < vertexids_firstmisson.size(); i++){
                        pose_graph::VertexId& vertexid = vertexids_firstmisson.at(i);
                        const vi_map::Vertex& cur_vertex = map->getVertex(vertexid);

                    }*/

                    // Every console command returns an integer, you can take one from
                    // the CommandStatus enum. kSuccess returns everything is fine.
                    // Other commonly used return values are common::kUnknownError and
                    // common::kStupidUserError.
                    return common::kSuccess;
                },
                // This is the description of your command. This will get printed when
                // you run `help` in the console.
                "This plugin is used to change map format",

                // This specifies the execution method of your command. For most
                // commands, it is sufficient to run them in sync with
                // common::Processing::Sync.
                common::Processing::Sync
                );
    }
};

// Finally, call the MAPLAB_CREATE_CONSOLE_PLUGIN macro to create your console
// plugin.
MAPLAB_CREATE_CONSOLE_PLUGIN(map_transpose);
