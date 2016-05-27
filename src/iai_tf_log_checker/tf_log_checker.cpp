#include <iai_tf_log_checker/json.hpp>
#include <iai_tf_log_checker/utils.hpp>
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>

using json = nlohmann::json;
using namespace iai_tf_log_checker;

std::vector<json> parseJsonEntries(const std::vector<std::string>& json_entries)
{
  std::vector<json> results;

  for(size_t i=0; i<json_entries.size(); ++i)
    try
    {
      json j = json::parse(json_entries[i]);
      results.push_back(j);
    }
    catch (const std::exception& e)
    {
      throw std::runtime_error("Could not parse json entry #" + 
        std::to_string(i) + ": " + e.what());
    }

  ROS_INFO("Successfully parsed %lu raw json entries.", json_entries.size());
  return results;
}
 
json safelyGetEntry(const json& entry, const std::string& entry_name, 
    const std::string& error_msg = "")
{
  if (entry.find(entry_name) == entry.end())
    throw std::runtime_error(error_msg + " JSON entry has no entry '" + entry_name + "'.");
  return entry[entry_name];
}

ros::Time toRosTime(const json& entry, const std::string& error_msg = "")
{
  size_t nsec = safelyGetEntry(entry, "$date", error_msg).get<size_t>();
  return ros::Time((double) nsec / 1000, (size_t) (nsec % 1000) * 1000000);
}

std_msgs::Header toHeaderMsg(const json& entry, const std::string& error_msg = "")
{
  std_msgs::Header msg;
  msg.frame_id = safelyGetEntry(entry, "frame_id", error_msg).get<std::string>();
  msg.stamp = toRosTime(safelyGetEntry(entry, "stamp", error_msg));
  msg.seq = safelyGetEntry(entry, "seq", error_msg).get<size_t>();

  return msg;
}

geometry_msgs::Vector3 toVector3Msg(const json& entry, const std::string& error_msg = "")
{
  geometry_msgs::Vector3 msg;
  msg.x = safelyGetEntry(entry, "x", error_msg).get<double>();
  msg.y = safelyGetEntry(entry, "y", error_msg).get<double>();
  msg.z = safelyGetEntry(entry, "z", error_msg).get<double>();
  return msg;
}

geometry_msgs::Quaternion toQuaternionMsg(const json& entry, const std::string& error_msg = "")
{
  geometry_msgs::Quaternion msg;
  msg.x = safelyGetEntry(entry, "x", error_msg).get<double>();
  msg.y = safelyGetEntry(entry, "y", error_msg).get<double>();
  msg.z = safelyGetEntry(entry, "z", error_msg).get<double>();
  msg.w = safelyGetEntry(entry, "w", error_msg).get<double>();
  return msg;
}

geometry_msgs::Transform toTransformMsg(const json& entry, 
    const std::string& error_msg = "")
{
  geometry_msgs::Transform msg;
  msg.translation = toVector3Msg(safelyGetEntry(entry, "translation", error_msg));
  msg.rotation = toQuaternionMsg(safelyGetEntry(entry, "rotation", error_msg));
  return msg;
}

geometry_msgs::TransformStamped toTransformStampedMsg(const json& entry, 
    const std::string& error_msg = "")
{
  geometry_msgs::TransformStamped msg;
  msg.header = toHeaderMsg(safelyGetEntry(entry, "header", error_msg), 
      error_msg);
  msg.child_frame_id = "TODO: parse me!";
  msg.transform = toTransformMsg(safelyGetEntry(entry, "transform", error_msg));
  return msg;
}

tf2_msgs::TFMessage toTfMessage(const json& json_entry, const std::string& error_msg = "")
{
  json transforms_entry = safelyGetEntry(json_entry, "transforms", error_msg);

  tf2_msgs::TFMessage msg;
  for(json::iterator it=transforms_entry.begin(); it != transforms_entry.end(); ++it)
    msg.transforms.push_back(toTransformStampedMsg(*it));
  return msg;
}

std::vector<tf2_msgs::TFMessage> toTfMessages(const std::vector<json>& json_entries)
{
  std::vector<tf2_msgs::TFMessage> msgs;
  for(size_t i=0; i<json_entries.size(); ++i)
    msgs.push_back(toTfMessage(json_entries[i], "Entry #" + std::to_string(i) + " could not be parsed to a TF message."));
  ROS_INFO("Conversion of into TF messages successful.");
  return msgs;
}

void checkTfJson(const std::vector<std::string>& raw_json_entries)
{
  std::vector<json> parsed_entries = parseJsonEntries(raw_json_entries);
  std::vector<tf2_msgs::TFMessage> msgs = toTfMessages(parsed_entries);
  std::cout << msgs[0].transforms[0] << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_log_checker");
  ros::NodeHandle nh("~");

  try
  {
    std::string url = "package://iai_tf_log_checker/test_data/pr2_no_odom.tf.json";
    checkTfJson(readFileLinesFromUrl(url));
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
    return 0;
  }

  return 0;
}
