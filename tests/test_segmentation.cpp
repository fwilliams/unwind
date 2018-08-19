#include "preprocessing.hpp"


int main(int argc, char *argv[]) {
  // need to call preprocessing using the data name
  std::string ipFolder = "D:/Fish_Deformation/Plagiotremus-tapinosoma";
  //std::string ipFolder= "/home/harishd/Desktop/Projects/Fish/data/straightening/OSF/Plagiotremus-tapinosoma";
  std::string filePrefix = "Plaagiotremus_tapinosoma_9.9um_2k__rec_Tra";
  std::string ext = "bmp";
  int stCt = 2;
  int enCt = 1798;

  std::string opFolder = "D:/Fish_Deformation/Plagiotremus-tapinosoma/output";
  std::string opPrefix = "Plaagiotremus_tapinosoma";

  SamplingOutput op = ImageData::writeOutput(ipFolder, filePrefix, stCt, enCt, ext, opFolder, opPrefix, 4, true);
  preProcessing(op.fileName, op.x, op.y, op.z);
  return EXIT_SUCCESS;
}
