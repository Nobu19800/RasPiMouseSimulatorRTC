#include <coil/stringutil.h>
#include "searchFile.h"

#ifdef WIN32
#include <shlwapi.h>
#else
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#endif



bool exist_file(const char* path)
{
#ifdef WIN32
	BOOL ans = PathFileExists(path);
	if(ans == TRUE)
	{
		return true;
	}
	else
	{
		return false;
	}
#else
	struct stat st;
	if (ret == 0) {
		if((st.st_mode & S_IFMT) != S_IFDIR)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return true;
	}
#endif
}

bool exist_directory(const char* path)
{
#ifdef WIN32
	BOOL ans = PathIsDirectory(path);
	if (ans == TRUE)
	{
		return true;
	}
	else
	{
		return false;
	}
#else
	struct stat st;
	if (ret == 0) {
		if ((st.st_mode & S_IFMT) == S_IFDIR)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return true;
	}
#endif
}


std::string search_file(const char* filename, const char* env, const char* delim)
{
	char* buf = 0;
	size_t sz = 0;
	std::string val = "";
	if (_dupenv_s(&buf, &sz, env) == 0)
	{
		if (!buf)
		{
			return val;
		}
		val = buf;


		delete buf;
		std::vector<std::string> split_txt = coil::split(val, delim);
		for (std::vector<std::string>::iterator it = split_txt.begin(); it != split_txt.end(); ++it) {
			std::string tmp = (*it);
			if(tmp.size() == 0)continue;
#ifdef WIN32
			if (tmp.substr(tmp.size() - 1) == "\\")
#else
			if (tmp.substr(tmp.size() - 1) == "/")
#endif

			{
				tmp.erase(tmp.end());
			}
#ifdef WIN32
			std::string path = tmp + "\\";
			
			
#else
			std::string path = tmp + "/";
			
#endif
			path = path + filename;

#ifdef WIN32
			coil::replaceString(path, "/", "\\");

#else
			coil::replaceString(path, "\\", "/");
#endif

			if (exist_file(path.c_str()))
			{
				return path;
			}
			else if (exist_directory(path.c_str()))
			{
				return path;
			}

		}
		return "";
	}
	else
	{
		return "";
	}
}