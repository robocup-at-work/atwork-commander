#pragma once

#define BOOST_DATE_TIME_POSIX_TIME_STD_CONFIG
#include <ros/ros.h>

#include "Definitions.h"

#include <array>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>

#include <boost/asio.hpp>
#include <std_msgs/Bool.h>

using run = std::vector<std::array<int, 4>>;

namespace atwork_refbox_ros {

class ReceiverNode
{
    public:
        /**
         * Ctor.
         */
        ReceiverNode(const ros::NodeHandle &nh);
        ReceiverNode(const ArenaDescription& arena, const TaskDefinitions& tasks);

        void readParameters();
        void readParameters(unsigned int objects, unsigned int tables, unsigned int decoys, unsigned int pickShelf,
                            unsigned int pickTT, unsigned int placeShelf, unsigned int placeTT, unsigned int placePPT,
                            unsigned int containerRed, unsigned int containerBlue, 
                            bool containerInShelf = false, bool containerOnTT = false, bool containerOnPPT = false);

        void initializeRobot();
        run generate_Final();

        /**
         * Handler for receive messages .
         *
         * This handler is called for receiving messages.
         * Main function in this node.
         *
         */

    private:
        std::vector<std::string> mTableMapping;
        std::vector<unsigned int> mTables;
        std::vector<unsigned int> mTables0;
        std::vector<unsigned int> mTables5;
        std::vector<unsigned int> mTables10;
        std::vector<unsigned int> mTables15;
        std::vector<unsigned int> mConveyors;
        std::vector<unsigned int> mPpts;
        std::vector<unsigned int> mShelfs;
        std::vector<unsigned int> mWaypoints;
        std::vector<unsigned int> mObjects;
        std::vector<unsigned int> mPptObjects;
        std::vector<size_t> mAllTables;      
        
        int mdiscipline;

        int toLocation(unsigned int loc, unsigned int type, bool bnt=false) const;


        /**  \brief insert a new stacked TF at table
         *   \param table the table the TF will be created at
         *   \return name of the newly created TF frame
         **/
        std::string insertStackedTF(unsigned int table);

        /** \brief insert a container type at table if none exists
         *  \param object to be placed int the container
         *  \param container the container type
         *  \param table the table  to insert the container at
         *  \return the id of the newly created container object
         **/
         int insertContainer(int object, int container, unsigned int table);

        /** \brief start the task **/
        void start();

        /**
         * ROS node handle.
         */
        ros::NodeHandle nh_;

        /**
         * Stores sequence of the bacon messages.
         */
        unsigned long seq_;

      	unsigned long cycle_;

        unsigned long pptObjCount;

				bool publishTask = true;

        ros::ServiceClient start_service_;
        
        std::map<std::string, int> paramBNT;
        std::map<std::string, int> paramBTT3;
        std::map<std::string, int> paramFinal;
        int estimatet_active;
        bool paramContainerInShelf;
        bool paramContainerOnPpt;
        bool paramContainerOnTurntable;
        bool paramFlexibleHeight;
        
        void generate_objects(run &tasks);
        
        void initialize_mAllTables();
        void initialize_validpicks(run &tasks);
        void initialize_picksleft();
        void initialize_mTableTypes();
        size_t write_types(size_t low, size_t up, size_t type);
        void update_validpicks();
        void debugAll(std::string info, run &tasks);
        
        run auto_task_creation();
        run generate_BNT();
        run generate_BTT3();
        
        size_t get_container_id(size_t table, size_t color);
        size_t shortest_list(size_t &min);
        
        std::vector<std::array<size_t, 3>> container_ids;
        std::vector<std::vector<size_t>> validpicks;
        std::vector<size_t> picksleft;
        std::unordered_map<size_t, size_t> mTableTypes;
        
        size_t tabletypes;
        
        static const size_t obj_id = 0;
        static const size_t src_id = 1;
        static const size_t dst_id = 2;
        static const size_t cont_id = 3;
        
        static const size_t blue = 0;
        static const size_t red = 1;
        
        static const size_t tables0_id = 0;
        static const size_t tables5_id = 1;
        static const size_t tables10_id = 2;
        static const size_t tables15_id = 3;
        static const size_t conveyors_id = 4;
        static const size_t ppts_id = 5;
        static const size_t shelfs_id = 6;
};

}
