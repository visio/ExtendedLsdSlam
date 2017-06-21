#ifndef FOLDERCONTROLLER_H
#define FOLDERCONTROLLER_H

#include <string>
#include <boost/filesystem.hpp>

class FolderController
{
public:
    // Constructor
    FolderController( std::string projectName, std::string base_path = "" );

    // Set new base path
    void setBasePath( std::string basePath );
    // Get base path
    std::string getGetBasePath() const;

    // Set new project name
    void setProjectName( std::string projectName );
    // Get project path
    std::string getGetProjectPath() const;

    // Make new folder
    std::string makeSubfolder(std::string name);
    // Get current subfolder
    std::string getCurrentSubfloder() const;

private:
    // Base path
    boost::filesystem::path m_oBasePath;
    // Project path
    boost::filesystem::path m_oProjectPath;
    // Current folder path
    boost::filesystem::path m_oCurrentSubfolderPath;

    // Project folder name
    std::string m_sProjectName;

};

#endif // FOLDERCONTROLLER_H
