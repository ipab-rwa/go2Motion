cmake_minimum_required(VERSION 3.15)
project(go2Motion LANGUAGES CXX)

# Chercher le package go2Reachability
find_package(go2Reachability REQUIRED)	
find_package(sl1m REQUIRED)	

message(STATUS "GO2REACHABILITY_CONSTRAINTS_DIR = ${GO2REACHABILITY_CONSTRAINTS_DIR}")

#~ add_executable(test_find main.cpp)

# Définit une macro C++ avec le chemin du dossier
#~ target_compile_definitions(test_find PRIVATE
    #~ GO2REACHABILITY_CONSTRAINTS_DIR="${GO2REACHABILITY_CONSTRAINTS_DIR}")
