#include "vtd.hpp"

void vtd_roadmark_Reader::init(){
    std::string csv_path("./../vtd/output_roadmark.csv");
    read_stream.open(csv_path.c_str(),std::ios::in);
    if(!read_stream){
        std::stringstream ss;
        ss<<"Fail to open "<<csv_path;
        WARNING_MSG_EXIT(ss.str());
    }
    getline(read_stream,line); //ignore header
}


bool vtd_roadmark_Reader::read_roadmark(){
    uint markid, markindex, markCount;
    float x,y,z;
    if(seperate_data())
        return true;
    else
        return false;
}

bool vtd_roadmark_Reader::seperate_data(){
    if(getline(read_stream,line)){
        std::cout<<line<<std::endl;
        return true;
    }
    else
        return false;
}


std::vector<VISUALIZER::LANE>& vtd_roadmark_Reader::get_roadmark(){ 
    if(!read_roadmark())
        vec_LANE.clear();

    return vec_LANE;
}

