#include "reference_calc.cpp"
#include "utils.h"
#include <stdio.h>

__global__
void rgba_to_greyscale(const uchar4* const rgbaImage,
                       unsigned char* const greyImage,
                       int numRows, int numCols)
{
  //TODO
  //Fill in the kernel to convert from color to greyscale
  //the mapping from components of a uchar4 to RGBA is:
  // .x -> R ; .y -> G ; .z -> B ; .w -> A
  //
  //The output (greyImage) at each pixel should be the result of
  //applying the formula: output = .299f * R + .587f * G + .114f * B;
  //Note: We will be ignoring the alpha channel for this conversion

  //First create a mapping from the 2D block and grid locations
  //to an absolute 2D location in the image, then use that to
  //calculate a 1D offset
  int tx = threadIdx.x + blockIdx.x * blockDim.x;
  int ty = threadIdx.y + blockIdx.y * blockDim.y;
  uchar4 image = rgbaImage[tx + ty * numCols];
  float R = image.x;
  float G = image.y;
  float B = image.z;
  float A = image.w;
  greyImage[tx + ty * numCols] = 0.299f * R + 0.587f * G + 0.114f * B;
}

void your_rgba_to_greyscale(const uchar4 * const h_rgbaImage, uchar4 * const d_rgbaImage,
                            unsigned char* const d_greyImage, size_t numRows, size_t numCols)
{
  int block_width = 16;
  int block_height = 16;
  int grid_width = numCols / block_width;
  if (grid_width * block_width < numCols)
      grid_width++;
  int grid_height = numRows / block_height;
  if (grid_height * block_height < numRows)
      grid_height++;
  
  const dim3 blockSize(block_width, block_height );  //TODO
  const dim3 gridSize( grid_width, grid_height );  //TODO
  rgba_to_greyscale<<<gridSize, blockSize>>>(d_rgbaImage, d_greyImage, numRows, numCols);
  
  cudaDeviceSynchronize(); checkCudaErrors(cudaGetLastError());
}
