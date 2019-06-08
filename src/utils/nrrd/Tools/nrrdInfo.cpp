// Created by Andre Aichert on Mon Apr 7th 2014

#include <iostream>
#include <fstream>

#include <NRRD/nrrd_image.hxx>

template <typename X>
void set_tags(const std::string& file, char **keyvalepairs, int narg)
{
	NRRD::Image<X> img(file);
	if (!img)
	{
		std::cerr << "Failed to load image data!! Aborting.\n";
		exit(3);
	}
	for (int i=0;i<narg;i+=2)
	{
		std::string key=keyvalepairs[i];
		std::string value=keyvalepairs[i+1];
		img.meta_info[key]=value;
		if (value=="--")
			img.meta_info.erase(img.meta_info.find(key));
	}
	img.save(file);
}

int main(int argc, char ** argv)
{
	// Print help text
	if (argc<2 || std::string(argv[1])=="--help" || std::string(argv[1])=="-h")
	{
		std::cout <<	"Read NRRD-headers and get or set keys.\n"
//						"Setting fields possible but discouraged.\n"
//						"This utility will not let you override \"sizes\" \"type\" and \"encoding\" fields.\n"
						"Usage:\n"
						"   nrrdInfo file.nrrd\n"
						"   nrrdInfo file.nrrd list\n"
						"   nrrdInfo file.nrrd get <key>\n"
						"   nrrdInfo file.nrrd set [<key> <value>]^n\n"
//						"   nrrdInfo file.nrrd set_field [<field> <value>]^n\n"
						"\n"
						"Example: Print the \"Projection Matrix\" key:\n"
						"   nrrdInfo file.nrrd get \"Projection Matrix\"\n"
						"Use \"--\" value to erase a key:\n"
						"   nrrdInfo file.nrrd get \"Some Key\" --\n";
		return 1;
	}
	
	// The NRRD file to work with
	std::string file=argv[1];
	std::map<std::string,std::string> hdr_fields;
	std::map<std::string,std::string> hdr_keys;
	int bytes=NRRD::parseHeader(file,hdr_fields,&hdr_keys);
	if (bytes<3 && argc!=2) return 1;

	// Print header
	if (argc==2)
	{
		std::cout << "NRRD header has " << bytes << " Bytes:\n";
		std::ifstream nrrd(file);
		std::string line;
		std::getline(nrrd,line);
		while (!line.empty() && nrrd)
		{
			std::cout << line << std::endl;
			std::getline(nrrd,line);
		}
		return 0;
	}

	// Action can be "list" "get" or "set"
	std::string action=argv[2];
	if (action=="list")
	{
		if (hdr_keys.empty())
			std::cerr << "No keys in NRRD header.\n";
		for (auto it=hdr_keys.begin();it!=hdr_keys.end();++it)
			std::cout << it->first << std::endl;
		return 0;
	}
	else
	{
		if (action!="get" && action!="set")
		{
			std::cerr << "Unknown action: " << action << "\nTry \"get\" \"set\" or \"list\"";
			return 2;
		}
		// get key1 [key2 ...]
		if (action=="get")
		{
			if (argc<4)
			{
				std::cerr << "Action get requires you to provide at least one key.\n";
				return 2;
			}
			for (int i=3;i<argc;i++)
			{
				std::string key=argv[i];
				auto it=hdr_keys.find(key);
				if (it==hdr_keys.end())
					std::cerr << "Warning: Key \"" << key << "\" does not exist in header.\n";
				else if (it->second=="")
					std::cerr << "Warning: Key \"" << key << "\" is empty.\n";
				else
					std::cout << it->second;
				std::cout << std::endl;

			}
		}
		// set key1 value1 [key2 value2 ...]
		if (action=="set")
		{
			if (argc<5)
			{
				std::cerr << "Action set requires you to provide at least one key and one value.\n";
				return 2;
			}
			if (argc%2==0)
			{
				std::cerr << "Action set requires you to provide key-value pairs.\nYou provided an uneven number of items.\n";
				return 2;
			}

			for (int i=3;i<argc;i+=2)
			{
				std::string type=hdr_fields["type"];
				// Select correct template based on type
				#define _DEFINE_TYPE_NO_BOOL
				#define _DEFINE_TYPE_NO_LONG
				#define _DEFINE_TYPE(X) if (type==#X) set_tags<X>(file,argv+3,argc-3);
				#include "GetSet/BaseTypes.hxx"
				#undef _DEFINE_TYPE_NO_LONG
				#undef _DEFINE_TYPE_NO_BOOL
			}
		}
	}
	return 0;
}
