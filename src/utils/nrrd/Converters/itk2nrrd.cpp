// Created by A. Aichert on Fri Mar 1st 2013
#include <iostream>
#include <limits>

#include <ITKUtils.hxx>
#include <GetSet/GetSetAutoGui.hxx>
#include <GetSet/GetSetCmdLine.hxx>

#include <NRRD/nrrd_image.hxx>

GETSET_GLOBAL_DICTIONARY

// Actual source
template <typename TImage>
void convert()
{
	std::string path_in=GetSet<>("File/Input");
	std::string path_out=GetSet<>("File/Output");
	if (path_out.empty())
	{
		path_out=path_in;
		splitRight(path_out,".");
		path_out=path_out+".nrrd";
	}

	bool normalize=GetSet<bool>("Quantization/Normalize");
	double bias=GetSet<double>("Quantization/Bias");
	double scale=GetSet<double>("Quantization/Scale");
	if (scale==0) scale=1;

	if (path_out.empty())
		GetSetAutoGUI::error("Writing File","Please specify an output file.");

	typedef itk::Image<double,TImage::ImageDimension> DoubleImage;
	itk::MetaDataDictionary dcm_dict;
	DoubleImage::Pointer img=ITKUtils::loadImage<DoubleImage>(path_in,&dcm_dict);
	if (!img)
		GetSetAutoGUI::error("Loading File", std::string("Failed to load input file ") + path_in);

	// for integral types, normalize to min/max rather than 0/1
	double typemin=0,typemax=1;
	std::string type=typeName<TImage::PixelType>();
	if (type!="float"&&type!="double")
	{
		typemin=std::numeric_limits<TImage::PixelType>::min();
		typemax=std::numeric_limits<TImage::PixelType>::max();
	}

	// Access raw image data
	int w=0,h=0,d=0,k=0;
	DoubleImage::PixelType *imgptr=0x0;
	ITKUtils::getRawImage(img,&imgptr,&w,&h,&d,&k);
	int n=w*h*d*k;
	int v[]={w,h,d,k};
	NRRD::Image<TImage::PixelType> out(v,TImage::ImageDimension);

	auto origin=img->GetOrigin();
	auto rotation=img->GetDirection();
	auto spacing=img->GetSpacing();
	// Set spacing
	for (int i=0;i<TImage::ImageDimension;i++)
		out.spacing(i)=spacing[i];

	// Model Matrix FIXME should work for n-D
	std::vector<double> m(9);
	m[0]=spacing[0]*rotation(0,0);	m[1]=spacing[1]*rotation(1,0);	m[2]=origin[0];
	m[3]=spacing[0]*rotation(0,1);	m[4]=spacing[1]*rotation(1,1);	m[5]=origin[1];
	m[6]=0;							m[7]=0;							m[8]=1;
	


	// Iterate over data, compute min/max and rescale
	if (normalize)
	{
		DoubleImage::PixelType min=imgptr[0],max=imgptr[0];
		GetSetAutoGUI::info("Converting...","Requantization...");
		for (int i=0;i<n;i++)
		{
			if (i%100000==0) GetSetAutoGUI::progress("Converting...",i,n*2);
			if (imgptr[i]<min)min=imgptr[i];
			if (imgptr[i]>max)max=imgptr[i];
		}
		GetSetAutoGUI::info("Converting...","Applying bias and scale...");
		double b=-min+bias;
		double s=1.0/max*scale;
		for (int i=0;i<n;i++)
		{
			if (i%100000==0) GetSetAutoGUI::progress("Converting...",n+i,n*2);
			out[i]=(TImage::PixelType)((imgptr[i]*s+b)*(typemax-typemin)+typemin);
		}
		GetSetAutoGUI::progress_hide("Converting...");
	}
	else if (bias!=0 || scale!=1)
	{
		GetSetAutoGUI::info("Converting...","Applying bias and scale...");
		for (int i=0;i<n;i++)
		{
			if (i%100000==0) GetSetAutoGUI::progress("Converting...",i,n);
			out[i]=(TImage::PixelType)(imgptr[i]*scale+bias);	
		}
		GetSetAutoGUI::progress_hide("Converting...");
	}
	else for (int i=0;i<n;i++) out[i]=(TImage::PixelType)imgptr[i];

	GetSetAutoGUI::info("Converting...","Saving...");

	for (auto it=dcm_dict.Begin();it!=dcm_dict.End();++it)
	{
		std::string tag=it->first;
		std::string label;
		if (!itk::GDCMImageIO::GetLabelFromTag(tag,label))
			label="?";
		auto item=dynamic_cast<itk::MetaDataObject<std::string>*>( it->second.GetPointer() ) ;
		if (!item) continue;
		std::string value=item->GetMetaDataObjectValue();
		out.meta_info[tag]=value;
	}

	out.meta_info["Model Matrix"]=vectorToString(m);


	if (!out.save(path_out)) // dcm_dict
		GetSetAutoGUI::error("Saving File",std::string("Failed to save image to file ")+path_out);
	else
		std::cout << "File " << path_out << " written.\n";

}

int main(int argc, char ** argv)
{
	// Define parameters
	GetSetGui::File("File/Input")
		.setExtensions("All Files (*)")
		.setDescription("Input File (all file types supported by ITK)")
		.setAttribute("CommandLineFlag","-i");
	GetSetGui::File("File/Output")
		.setExtensions("nrrd (*.nrrd);;All Files (*)")
		.setCreateNew(true)
		.setDescription("Name of output, If more than one input file has been selected, a three digit file index will be appended.")
		.setAttribute("CommandLineFlag","-o")
		.setDescription("Output File (.nrrd)");
	GetSetGui::Enum("File/Dimension")
		.setChoices("2D;3D;4D").setDescription("2D, 3D or 4D data")
		.setAttribute("CommandLineFlag","-d")
		.setValue(1); // defaults to 3D
	GetSetGui::Enum("File/Type")
		.setChoices("unsigned char;char;unsigned short;short;unsigned int;int;float;double")
		.setDescription("float (default), double, char short int, unsigned *")
		.setAttribute("CommandLineFlag","-t;--type")
		.setString("float");

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
					"   itk2nrrd --xml\n"
					"   itk2nrrd config.ini\n"
					"   itk2nrrd file_in [-o file_out] [-t <type>]\n"
					"\n"
					"with: <type>:   "<<GetSet<>("File/Type").getDescription()<<"\n"
					"\n"
					<< cmd.getSynopsis();
					;
		return 0;
	}
	else if (!cmd.parse(argc, argv))
		GetSetAutoGUI::error("Commad Line", "Failed to parse command line arguments. Try: \"itk2nrrd --help\"");
	if (cmd.getUnhandledArgs().size()>1)
		GetSetAutoGUI::error("Commad Line", "Unrerecognized command line arguments. Try: \"itk2nrrd --help\"");


	// Select correct template based on type string and dimension
	int dim=GetSet<int>("File/Dimension")+2;
	std::string type=GetSet<>("File/Type");
	#define _DEFINE_TYPE_NO_BOOL
	#define _DEFINE_TYPE_NO_LONG
	if (dim==2)
	{
		#define _DEFINE_TYPE(X) if (type==#X) convert<itk::Image<X,2> >();
		#include "GetSet/BaseTypes.hxx"
		#undef _DEFINE_TYPE
	}
	else if (dim==3)
	{
		#define _DEFINE_TYPE(X) if (type==#X) convert<itk::Image<X,3> >();
		#include "GetSet/BaseTypes.hxx"
		#undef _DEFINE_TYPE
	}
	else if (dim==4)
	{
		#define _DEFINE_TYPE(X) if (type==#X) convert<itk::Image<X,4> >();
		#include "GetSet/BaseTypes.hxx"
		#undef _DEFINE_TYPE
	}
	else
		GetSetAutoGUI::error("Commad Line", "Only 2D, 3D and 4D images are supported by this tool. Try: \"itk2nrrd --help\"");
 
	return 0;
}
