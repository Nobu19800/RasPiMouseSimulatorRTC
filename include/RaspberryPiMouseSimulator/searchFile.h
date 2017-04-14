#ifndef SEARCHFILE_H
#define SEARCHFILE_H

#include <string>

std::string search_file(const char* filename, const char* env, const char* delim);

bool exist_file(const char* path);
bool exist_directory(const char* path);

#endif