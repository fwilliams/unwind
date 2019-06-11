// Created by A. Aichert on Fri Mar 1st 2013
#include <ITKUtils.hxx>
#include <GetSet/GetSetAutoGui.hxx>
#include <GetSet/GetSetCmdLine.hxx>
GETSET_GLOBAL_DICTIONARY

#include <NRRD/nrrd_image.hxx>

// Actual source
template <typename Type>
void convert()
{
	std::string path_in=GetSet<>("File/Input");
	std::string path_out=GetSet<>("File/Output");
	if (path_out.empty())
	{
		path_out=path_in;
		splitRight(path_out,".");
		path_out=path_out+".tiff";
	}
	NRRD::Image<Type> in(path_in);
	if (!in)
		GetSetAutoGUI::error("File Access Error", std::string("Cannot read NRRD image file ")+path_in);

	switch (in.dimension())
	{
	default:
		GetSetAutoGUI::error("Error",std::string("Unsupported number of dimensions: ")+toString(in.dimension()));
		break;
	case 2:
		{
			int v[]={in.size(0),in.size(1)};
			double s[]={in.spacing(0),in.spacing(1)};
			auto out=ITKUtils::createImage<Type,2>(v,in);
			out->SetSpacing(s);
			if (!ITKUtils::saveImage(out,path_out))
				GetSetAutoGUI::error("File Access Error", std::string("Cannot write image file ")+path_out);
		}
		break;
	case 3:
		{
			int v[]={in.size(0),in.size(1),in.size(2)};
			double s[]={in.spacing(0),in.spacing(1),in.spacing(2)};
			auto out=ITKUtils::createImage<Type,3>(v,in);
			out->SetSpacing(s);
			if (!ITKUtils::saveImage(out,path_out))
				GetSetAutoGUI::error("File Access Error", std::string("Cannot write image file ")+path_out);
		}
		break;
	case 4:
		{
			int v[]={in.size(0),in.size(1),in.size(2),in.size(3)};
			double s[]={in.spacing(0),in.spacing(1),in.spacing(2),in.spacing(3)};
			auto out=ITKUtils::createImage<Type,4>(v,in);
			out->SetSpacing(s);
			if (!ITKUtils::saveImage(out,path_out))
				GetSetAutoGUI::error("File Access Error", std::string("Cannot write image file ")+path_out);
		}
		break;
	}
}

int main(int argc, char ** argv)
{
	// Define parameters
	GetSetGui::File("File/Input")
		.setExtensions("NRRD Image (*.nrrd);;All Files (*)")
		.setDescription("Input File (NRRD image)") 
		.setAttribute("CommandLineFlag","-i");
	GetSetGui::File("File/Output")
		.setExtensions("nrrd (*.nrrd);;All Files (*)")
		.setCreateNew(true)
		.setDescription("Name of output (all file types supported by ITK inferred by extension)")
		.setAttribute("CommandLineFlag","-o");

	GetSetIO::CmdLineParser cmd;
	cmd.index("File/Input",1);
	cmd.declare();

	// check if we have an ini-file
	std::string a1ext=argc==2?std::string(argv[1]):"";
	bool is_ini_file=splitRight(a1ext,".")=="ini";

	// Handle --xml command line argument and ini-File
	if (argc==2 && std::string(argv[1])=="--xml")
	{
		std::cout << GetSetAutoGUI::getXML();
		return 0;
	}
	else if (is_ini_file)
		GetSetAutoGUI::loadIni(argv[1]);
	else if ( argc==1 || argc==2 && (std::string(argv[1])=="-h" || std::string(argv[1])=="--help" ))
	{
		std::cerr
				<<	"Usage:\n"
					"   nrrd2itk --xml\n"
					"   nrrd2itk config.ini\n"
					"   nrrd2itk file_in [-o file_out]\n"
					"Converts to tiff by default.\n"
					<< cmd.getSynopsis();
					;
		return 0;
	}
	else if (!cmd.parse(argc, argv))
		GetSetAutoGUI::error("Commad Line", "Failed to parse command line arguments. Try: \"nrrd2itk --help\"");
	if (cmd.getUnhandledArgs().size()>1)
		GetSetAutoGUI::error("Commad Line", "Unrerecognized command line arguments. Try: \"nrrd2itk --help\"");

	// Get dimension and type
	std::string file_in=GetSet<>("File/Input");
	std::map<std::string,std::string> hdr;
	if (NRRD::parseHeader(file_in,hdr)<5)
		GetSetAutoGUI::error("File Access Error", std::string("Cannot read NRRD header from file ")+file_in);

	// By default, convert to tiff
	if (argc==2)
	{
		std::string file=GetSet<>("File/Input");
		splitRight(file,".");
		GetSet<>("File/Output")=file+".tif";
	}

	// Select correct template based on type string and dimension
	std::string type=hdr["type"];
	#define _DEFINE_TYPE_NO_BOOL
	#define _DEFINE_TYPE_NO_LONG
	#define _DEFINE_TYPE(X) if (type==#X) convert<X>();
	#include "GetSet/BaseTypes.hxx"



	return 0;
}
