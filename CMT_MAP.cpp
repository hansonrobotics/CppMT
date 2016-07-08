#include "CMT_MAP.h"
#include <algorithm>
namespace cmt {
void CMTMAP::process(const Mat im_gray, const int factor,std::vector<string> string_)
{
    //TODO make this one a thread
 for(std::vector<std::string>::iterator v=string_.begin(); v!= string_.end();v++)
 {
    if(cmt_[*v].initialized)
    cmt_[*v].processFrame(im_gray, factor);
 }
}
std::vector<cmt_message> CMTMAP::process_map(const Mat im_gray, const double factor,std::map<std::string, std::string> merge, int frame_wait)
{
std::vector<cmt_message> cmt_messages;
queue_tracker.clear();

separate();

boost::thread thread_1 = boost::thread(&CMTMAP::process,this,im_gray, factor,string_1);
boost::thread thread_2 = boost::thread(&CMTMAP::process,this,im_gray, factor,string_2);
boost::thread thread_3 = boost::thread(&CMTMAP::process,this,im_gray, factor,string_3);
boost::thread thread_4 = boost::thread(&CMTMAP::process,this,im_gray, factor,string_4);

thread_4.join();
thread_3.join();
thread_2.join();
thread_1.join();

newFaces.clear();
lostFaces.clear();

//TODO Better the merging process with update_area fucntions;
for(std::map<std::string, std::string>::iterator v = merge.begin(); v!= merge.end(); v++)
{
    queue_tracker.push_back(v->second);
    lostFaces.push_back(v->second);
}

for(std::map<std::string, cmt::CMT>::iterator v = cmt_.begin(); v!= cmt_.end(); v++)
{
  cmt_message message;
//  v->second.processFrame(im_gray, factor);
  message.initial_active_points = v->second.num_initial_keypoints;
  message.active_points = v->second.num_active_keypoints;
  message.tracker_name = v->second.name;
  cv::Rect rect = v->second.bb_rot.boundingRect();

  message.rect = rect & cv::Rect(0, 0, im_gray.size().width, im_gray.size().height);
//  std::cout<<"What is lost: "<<v->second.tracker_lost<<std::endl;
  double division = (double) message.active_points / (double)message.initial_active_points;


//  std::cout<<"initial_active_points: "<<message.initial_active_points<<std::endl;
//  std::cout<<"active_points: "<<message.active_points<<std::endl;
//  std::cout<<"division: "<<division<<std::endl;

  if(division > factor)
  {
  message.tracker_lost = false;
  //std::cout<<"Frame Wait: "<<frame_wait<<std::endl;
  v->second.ratio_frames = frame_wait; //TODO dynamic parameters.
  }
  else
  {
      if(v->second.ratio_frames == 0)
      {
      message.tracker_lost = true;
      std::cout<<"Lost by Lower ration for 10 frames"<<std::endl;
      }
      else
      {
      message.tracker_lost = false;
      v->second.ratio_frames = v->second.ratio_frames - 1;
      std::cout<<"Ratio not being met"<<std::endl;
      }
  }
  if( v->second.decreasing_validate == v->second.initial_default)
  {
      message.tracker_lost = true;
      std::cout<<"Decreasing Validated reached lower threshold"<<std::endl;
  }
  message.validated = v->second.validated;
  face_reg[message.tracker_name] = message.validated;
  //Only create this if there is that index in the system.
  bool updated = false;
  if (face_reg_back.find(message.tracker_name)!=face_reg_back.end())
    {
    if (face_reg_back[message.tracker_name] != face_reg[message.tracker_name])
    {
        if (face_reg[message.tracker_name])
        {
            face_reg_back[message.tracker_name] = true;
            face_reg[message.tracker_name] = true;
            newFaces.push_back(message.tracker_name);
         }
       else
        {
        updated = true;
        lostFaces.push_back(message.tracker_name);
        face_reg_back[message.tracker_name] = false;
        face_reg[message.tracker_name] = false;
        }
    }
    }
     //Event trigger here
  message.before_being_demoted = v->second.decreasing_validate;

  if(message.tracker_lost)
  {
  //TODO There needs to a logic to handle this as quickly removed trackers are not particulaly good.
    if (!updated)
    {
    lostFaces.push_back(message.tracker_name);
    }
  queue_tracker.push_back(message.tracker_name);
  }
  message.recognized = v->second.identified;
  message.recognized_as = v->second.recognized_as;
  cmt_messages.push_back(message);

}

return cmt_messages;
}
int CMTMAP::size_map()
{
    return cmt_.size();
}
void CMTMAP::removeLost()
{
  //TODO this needs to be a state machines and conditions to not start deleting trackers that where started to track below threshold. So Flexiable threshold.

  for(std::vector<string>::iterator v = queue_tracker.begin(); v!= queue_tracker.end(); v++)
  {
    cmt_.erase(*v);
    face_reg.erase(*v);
    face_reg_back.erase(*v);
    newFaces.erase(std::remove(newFaces.begin(), newFaces.end(), *v), newFaces.end());
  }

}
std::vector<string> CMTMAP::newFace()
{
    return newFaces;  //Now this is face that are validated;
}
std::vector<string> CMTMAP::lostFace()
{
  for(std::vector<string>::iterator v = lostFaces.begin(); v!= lostFaces.end(); v++)
  {
    newFaces.erase(std::remove(newFaces.begin(), newFaces.end(), *v), newFaces.end());
  }
  return lostFaces;  //Now this is face that are validated;
}
void CMTMAP::clearFace()
{
    newFaces.clear();
    lostFaces.clear();
}

std::map<string, Mat> CMTMAP::getImages()
{
std::map<string, Mat> returnImages;
 for(std::map<std::string, cmt::CMT>::iterator v = cmt_.begin(); v!= cmt_.end(); v++)
 {
    if(v->second.validated)
        returnImages[v->first] = v->second.imArchive;
 }
 return returnImages;
}
void CMTMAP::updateArea(const Mat im_gray, cv::Rect area_, string name)
{

cmt_[name].updateArea(im_gray, area_);

}
void CMTMAP::deleteTracker(std::string name)
{
    cmt_.erase(name);
}
string CMTMAP::addtomap(const Mat im_gray,const Rect rect)
{
  int tracker_num = 0;
  srand(time(NULL));
  //TODO Fix this to not contain conflicts going forward in previously saved faces
  //and in the files that exist here.

  while (tracker_num == 0)
  {
  tracker_num = rand() % 100000;
  }
  std::string tracker_name = SSTR(tracker_num);
  //TODO Here we need to do a check to remove unresolved names
  cmt_[tracker_name] = cmt::CMT();
  cmt_[tracker_name].consensus.estimate_rotation = true;
  cmt_[tracker_name].initialize(im_gray, rect, tracker_name);
  cmt_[tracker_name].validated = false;
  face_reg_back[tracker_name] = false;
  face_reg[tracker_name] = false;
  return tracker_name;
}

bool CMTMAP::updatemapname(string tempname, int index)
{
if (cmt_.find(SSTR(index)) == cmt_.end())
{
  cmt_[tempname].set_name(SSTR(index));
  return true;
}
else
{
  //TODO handle if there is a name where there is an index of a name.
return false;
}
}

void CMTMAP::clear()
{
//Now remove the trackers just as you would.
 lostFaces.clear();
for(std::map<std::string, cmt::CMT>::iterator v = cmt_.begin(); v!= cmt_.end(); v++)
{
    if((*v).second.validated)
    lostFaces.push_back((*v).first);
}
cmt_.clear();
}

bool CMTMAP::validate(string name)
{
    cmt_[name].validated = true;
    return true;
}

bool CMTMAP::reinforce(string name, int value)
{
    if (cmt_.find(name)!=cmt_.end())
    {
    cmt_[name].reset_decreasing_validate(value);
    cmt_[name].validated = true;

    }
    return true;
}
void CMTMAP::separate()
{
int next = 0;
string_1.clear();
string_2.clear();
string_3.clear();
string_4.clear();
for(std::map<std::string, cmt::CMT>::iterator v = cmt_.begin(); v!= cmt_.end(); v++)
{

    if(next == 0)
    {
        string_1.push_back(v->first);
        next++;
    }
    else if(next == 1)
    {
        string_2.push_back(v->first);
        next++;
    }
    else if(next == 2)
    {
        string_3.push_back(v->first);
        next++;
    }
    else
    {
        string_4.push_back(v->first);
        next = 0;
    }
}
}

}