#ifndef IAI_TF_LOG_CHECKER_UTILS_H
#define IAI_TF_LOG_CHECKER_UTILS_H
     
#include <string>
#include <cstring>
#include <exception>
#include <ros/package.h>
#include <fstream>

namespace iai_tf_log_checker
{
  std::string assembleErrorMsg(const std::string& file, const std::string& error_msg)
  {
    return "Error retrieving file [" + file + "]: " + error_msg;
  }

std::string resourceUrlToPath(const std::string& url)
  {
    // TODO: use a real parser (like the boost one I used for the KMS40 driver).
    std::string path = url;
    if (!url.find("package://") == 0)
      throw std::runtime_error(assembleErrorMsg(url, "Given url does not start with 'package://'."));
    
    path.erase(0, std::strlen("package://"));
    size_t pos = path.find("/");
    if (pos == std::string::npos)
      throw std::runtime_error(assembleErrorMsg(url, "Could not find package name."));
  
    std::string package = path.substr(0, pos);
    path.erase(0, pos);
    std::string package_path = ros::package::getPath(package);
    if (package_path.empty())
      throw std::runtime_error(assembleErrorMsg(url, "Package '" + package + "' does not exist."));
  
    return package_path + path;
  }

  std::vector<std::string> readFileLinesFromPath(const std::string& path)
  {
    std::string line;
    std::vector<std::string> result;
    std::ifstream file(path);
    if (!file.is_open())
      throw std::runtime_error("Could not open file at '" + path + "'.");

    

    while ( std::getline (file,line) )
      result.push_back(line);
      //result += line + "\n";
    file.close();

    return result;
  }

  std::string readFileFromPath(const std::string& path)
  {
    std::vector<std::string> lines = readFileLinesFromPath(path);
    std::string result;
    for(size_t i=0; i<lines.size(); ++i)
      result += lines[i];
    return result;
  }

  std::vector<std::string> readFileLinesFromUrl(const std::string& url)
  {
    return readFileLinesFromPath(resourceUrlToPath(url));
  }

  std::string readFileFromUrl(const std::string& url)
  {
    return readFileFromPath(resourceUrlToPath(url));
  }

}
#endif // IAI_TF_LOG_CHECKER_UTILS_H
