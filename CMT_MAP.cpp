#include "CMT_MAP.h"

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
std::vector<cmt_message> CMTMAP::process_map(const Mat im_gray, const int factor,std::map<std::string, std::string> merge, double ratio)
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

//TODO Better the merging process with update_area fucntions;
for(std::map<std::string, std::string>::iterator v = merge.begin(); v!= merge.end(); v++)
{
    queue_tracker.push_back(v->second);
}
merge.clear();
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



  if(division > ratio)
  {
  message.tracker_lost = false;
  v->second.ratio_frames = 10; //TODO dynamic paramters.
  }
  else
  {
      if(v->second.ratio_frames == 0)
      {
      message.tracker_lost = true;
      }
      else
      {
      message.tracker_lost = false;
      v->second.ratio_frames = v->second.ratio_frames - 1;
      }
  }
  if( v->second.decreasing_validate == v->second.initial_default)
  {
      message.tracker_lost = true;
  }
  message.validated = v->second.validated;
  message.before_being_demoted = v->second.decreasing_validate;

  if(message.tracker_lost)
  {
  //TODO There needs to a logic to handle this as quickly removed trackers are not particulaly good.
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
std::vector<string> CMTMAP::removeLost()
{
  //TODO this needs to be a state machines and conditions to not start deleting trackers that where started to track below threshold. So Flexiable threshold.

  for(std::vector<string>::iterator v = queue_tracker.begin(); v!= queue_tracker.end(); v++)
  {
    cmt_.erase(*v);
  }
  return queue_tracker;
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
string CMTMAP::addtomap(const Mat im_gray,const Rect rect)
{
  int tracker_num;
  srand(time(NULL));
  //TODO Fix this to not contain conflicts going forward in previously saved faces
  //and in the files that exist here.
  tracker_num = rand() % 100000;
  std::string tracker_name = SSTR(tracker_num);
  //TODO Here we need to do a check to remove unresolved names
  cmt_[tracker_name] = cmt::CMT();
  cmt_[tracker_name].consensus.estimate_rotation = true;
  cmt_[tracker_name].initialize(im_gray, rect, tracker_name);
  cmt_[tracker_name].validated = false;
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
 queue_tracker.clear();
for(std::map<std::string, cmt::CMT>::iterator v = cmt_.begin(); v!= cmt_.end(); v++)
{
queue_tracker.push_back((*v).first);
}

}

bool CMTMAP::validate(string name)
{
    cmt_[name].validated = true;
    return true;
}

bool CMTMAP::reinforce(string name, int value)
{
    cmt_[name].reset_decreasing_validate(value);
    cmt_[name].validated = true;
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