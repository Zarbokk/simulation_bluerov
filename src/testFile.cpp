//
// Created by tim on 26.03.21.
//

#include <slamToolsRos.h>
#include <hilbertMap.h>
#include "json.h"

visualization_msgs::MarkerArray createMarkerArrayOfHilbertMap(hilbertMap &currentHilbertMap,double threshholdOccupancy = 0.15){
    int pointsDimensionInMap = currentHilbertMap.getNumberOfPointsForEachDimension();
    double sizeOfEachCell = currentHilbertMap.getDiscritisationSize();
    visualization_msgs::MarkerArray arrayOfCells;
    for(int i = 0 ; i<pointsDimensionInMap;i++){
        for(int j = 0 ; j<pointsDimensionInMap;j++){
            visualization_msgs::Marker tmpMarker;
            tmpMarker.pose.position.x = i*sizeOfEachCell-0.5*sizeOfEachCell*(pointsDimensionInMap-1);
            tmpMarker.pose.position.y = j*sizeOfEachCell-0.5*sizeOfEachCell*(pointsDimensionInMap-1);
            tmpMarker.pose.position.z = 0;
            tmpMarker.pose.orientation.w = 1;
            tmpMarker.header.frame_id = "map_ned";
            tmpMarker.scale.x = sizeOfEachCell;
            tmpMarker.scale.y = sizeOfEachCell;
            tmpMarker.scale.z = sizeOfEachCell;
            //determine color:
            if(currentHilbertMap.getCurrentMap()[i][j]>0.5+threshholdOccupancy){
                tmpMarker.color.r = 1;
                tmpMarker.color.g = 0;
                tmpMarker.color.b = 0;
                tmpMarker.color.a = 0.8;
            }else{
                if (currentHilbertMap.getCurrentMap()[i][j]<0.5-threshholdOccupancy){
                    tmpMarker.color.r = 0;
                    tmpMarker.color.g = 1;
                    tmpMarker.color.b = 0;
                    tmpMarker.color.a = 0.2;
                }else{
                    tmpMarker.color.r = 0;
                    tmpMarker.color.g = 0;
                    tmpMarker.color.b = 1;
                    tmpMarker.color.a = 0.4;
                }
            }

            //tmpMarker.lifetime.sec = 10;

            tmpMarker.type = 1;//1 for cube
            tmpMarker.id = i*pointsDimensionInMap+j;
            //i++;
            arrayOfCells.markers.push_back(tmpMarker);

        }
    }
    return arrayOfCells;
}

void readGraphSlamJson(std::string fileName){
    Json::Value keyFrames;
    std::ifstream keyFramesFile(fileName, std::ifstream::binary);
    keyFramesFile >> keyFrames;
    std::cout<<keyFrames["keyFrames"][0]["position"];//
    std::cout << "\n";

    //test to double
    double x = keyFrames["keyFrames"][0]["position"]["x"].asDouble();
    std::cout << "only x: "<< keyFrames["keyFrames"].size() << std::endl;

}


int
main(int argc, char **argv) {
    ros::init(argc, argv, "createHilbertMap");
    ros::start();
    ros::NodeHandle n_;
    ros::Publisher publisherMarkerArray;
    publisherMarkerArray = n_.advertise<visualization_msgs::MarkerArray>("occupancyHilbertMap", 10);

    hilbertMap mapRepresentation(150,0.2);
    mapRepresentation.createRandomMap();

    ros::Rate loop_rate(1);

//    while (ros::ok()){
//        visualization_msgs::MarkerArray markerArrayOfMap = createMarkerArrayOfHilbertMap(mapRepresentation);
//        publisherMarkerArray.publish(markerArrayOfMap);
//        ros::spinOnce();
//        loop_rate.sleep();
//    }

    readGraphSlamJson("/home/tim/DataForTests/DataFromSimulationJson/testfile.json");

    return (0);
}
