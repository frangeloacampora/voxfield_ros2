#include <memory>
#include <string>

#include <glog/logging.h>

${1}voxfield/core/block.h"
${1}voxfield/core/layer.h"
${1}voxfield/core/voxel.h"
${1}voxfield/io/layer_io.h"

using namespace voxfield;  // NOLINT

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  if (argc != 2) {
    throw std::runtime_error("Args: filename to load");
  }

  const std::string file = argv[1];

  Layer<EsdfVoxel>::Ptr layer_from_file;
  io::LoadLayer<EsdfVoxel>(file, &layer_from_file);

  LOG(INFO) << "Layer memory size: " << layer_from_file->getMemorySize();

  return 0;
}
