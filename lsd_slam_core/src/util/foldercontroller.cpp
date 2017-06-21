#include "foldercontroller.h"

#include <iostream>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>

FolderController::FolderController(std::string projectName, std::string base_path ) :
    m_sProjectName  ( projectName )
{
    // Configure environment
    setBasePath( base_path );
}

void FolderController::setBasePath(std::string basePath)
{
    // if base path not set
    if( basePath.empty() )
    {
        // set path to application by defsult
        m_oBasePath = boost::filesystem::path( boost::filesystem::current_path() );
    }
    else
    {
        // set input path
        m_oBasePath = boost::filesystem::path( basePath );

        // Check is directory exist
        if( !boost::filesystem::is_directory( m_oBasePath ) )
        {
            std::cout << "Path does't exist: " << m_oBasePath << std::endl;

            // Set path by default
            m_oBasePath = boost::filesystem::path( boost::filesystem::current_path() );
        }
    }

    // Create project path
    setProjectName(m_sProjectName);
}

std::string FolderController::getGetBasePath() const
{
    return m_oBasePath.string();
}

void FolderController::setProjectName(std::string projectName)
{
    // Project name
    m_sProjectName  = projectName;

    // Append project name to base path
    m_oProjectPath = m_oBasePath /= projectName;

    // Check is directory exist
    if( !boost::filesystem::is_directory( m_oProjectPath ) )
    {
        std::cout << "Create directory: " << m_oProjectPath << std::endl;

        // Create project directory
        if ( !boost::filesystem::create_directory( m_oProjectPath ) )
            std::cout << "Error: Create directory: " << m_oProjectPath << std::endl;
    }

    // Show selected path
    std::cout << "Current project path is: " << m_oProjectPath << std::endl;
}

std::string FolderController::getGetProjectPath() const
{
    return m_oProjectPath.string();
}

std::string FolderController::makeSubfolder(std::string name)
{
    // return val
    std::string res("");

    // default iterator for end
    boost::filesystem::directory_iterator end_itr;
    // temp string sublolder name
    std::string                 tempSubfolder;
    std::vector<std::string>    tempVector;

    // Numbers
    int currentNumber;
    int newNumber;
    currentNumber = newNumber = 0;

    // cycle through the directory
    for ( boost::filesystem::directory_iterator itr(m_oProjectPath); itr != end_itr; ++itr )
    {
        // if it is directory
        if( boost::filesystem::is_directory( itr->status() ))
        {
            // if subfolder containe name
            tempSubfolder = itr->path().filename().string();

            // split string
            boost::split( tempVector, tempSubfolder, boost::is_any_of("_") );

            // check components
            if( tempVector.size() < 2 || tempVector[0] != name )
                continue;

            // Recieve new number
            newNumber = std::stoi( tempVector[1] );

            if( newNumber >= currentNumber  )
                currentNumber = newNumber;
        }
    }

    // Make new full name
    std::string newFolderPath(m_oProjectPath.string());
    // append number
    newFolderPath = newFolderPath + name + "_" + std::to_string( ++currentNumber ) + "/";

    // Create new subdirectory directory
    if ( !boost::filesystem::create_directory( newFolderPath ) )
    {
        std::cout << "Error: Create directory: " << newFolderPath << std::endl;
        return res;
    }

    m_oCurrentSubfolderPath = boost::filesystem::path(newFolderPath);

    std::cout << "New subfolder: " << newFolderPath << std::endl;

    return m_oCurrentSubfolderPath.string();


    // To save the file names in a vector.
//    std::vector<boost::filesystem::directory_entry> v;

//    boost::filesystem::copy( boost::filesystem::directory_iterator( m_oProjectPath),
//                             boost::filesystem::directory_iterator(),
//                             boost::filesystem::back_inserter(v));

//    std::cout << p << " is a directory containing:\n";

//    for ( std::vector<directory_entry>::const_iterator it = v.begin(); it != v.end();  ++ it )
//    {
//        std::cout << (*it).path().string()<<endl;
    //    }
}

std::string FolderController::getCurrentSubfloder() const
{
    return m_oCurrentSubfolderPath.string();
}
