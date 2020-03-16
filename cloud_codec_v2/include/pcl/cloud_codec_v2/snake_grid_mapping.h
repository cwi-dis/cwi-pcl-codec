#ifndef SNAKE_GRID_MAPPING_H
#define SNAKE_GRID_MAPPING_H

#include <stdio.h>
#include <vector>
#include <stdint.h>

namespace pcl{

  namespace octree{    
     /**!
    \brief class for doing mapping of vector of rgb values into an image grid and back via a zigzag pattern
    \author Rufael Mekuria rufael.mekuria@cwi.nl
    */
    template<typename cTypeIn=std::uint8_t,typename cTypeOut=std::uint8_t>
    class SnakeGridMapping
    {
    public:
      /*!
      \brief helper class to iterate over the image grid in a zigzag order
      \author Rufael Mekuria rufael.mekuria@cwi.nl
      */
      class SnakeGridIterator
      {
      public:
        //! 
        SnakeGridIterator(int dim_w, int dim_h)
          : dim_w_(dim_w) , dim_h_(dim_h)
        {	
          w_pos = 0;
          h_pos = 0;
          macro_block_pos_w = 0;
          macro_block_pos_h = 0;
          to_right = true;
        }

        void reset(){
          w_pos = 0;
          h_pos = 0;
          macro_block_pos_w = 0;
          macro_block_pos_h = 0;
          to_right = true;
        };

        //! helper for forward iteration
        void updatePos(){
          to_right ? w_pos++ : w_pos--;
          if(((w_pos % 8 == 0) && to_right) || w_pos < 0 ){
            h_pos++;
            to_right = ! to_right;
            if(to_right)
              w_pos = 0;
            else
              w_pos = 7;
            if(h_pos % 8 == 0 || (h_pos + macro_block_pos_h * 8 == dim_h_)){
              h_pos=0;
              macro_block_pos_w++;
              if(macro_block_pos_w % (dim_w_ / 8) == 0){
                macro_block_pos_w = 0;
                macro_block_pos_h++;
              }
            }
          }
        }

        //! forward iterator
        int operator++(int){
          int ret = (w_pos + (h_pos + 8 *macro_block_pos_h) * dim_w_ + macro_block_pos_w * 8);
          updatePos();
          return ret;
        }

      protected:
        int dim_w_,dim_h_;
        int w_pos;
        int h_pos;
        int macro_block_pos_w;
        int macro_block_pos_h;
        bool to_right;
      };

      /**!
      \brief creates a class for generating a mapping to the RGB values based on a pattern
      */
      SnakeGridMapping(int dim_w, int dim_h)
        : dim_w_(dim_w) , dim_h_(dim_h), l_it(dim_w,dim_h)
      {	
      };

      /**!
      \brief simple helper to display the mapping
      */
      void printMapping(){
        for(int i=0; i<(result_.size()/3);i++)
        {
          if(i % dim_w_ == 0)
            printf("\n");
          printf(" %3d, ",result_[3 * i]);	
        }
      }

      /**!
      \brief map to an image grid trying to exploit spatial correlations
      */
      std::vector<cTypeOut> &doMapping(std::vector<cTypeIn> & input_data){
        //resize the output vector
        result_.resize(3 * dim_w_ * dim_h_);
        // start the iterator
        l_it.reset();

        for(int i =0; i < dim_w_*dim_h_; i++ ){
            int l_pos = l_it++;
          result_[3 * (l_pos) + 0] = input_data[3*i];
          result_[3 * (l_pos) + 1] = input_data[3*i + 1];
          result_[3 * (l_pos) + 2] = input_data[3*i + 2];
        }
        return result_;
      }

      /**!
      \brief undo map to an image grid trying to exploit spatial correlations
      */
      std::vector<cTypeOut> &undoSnakeGridMapping(std::vector<cTypeIn> &input_data){
        // initialize output and iterator
        result_.resize(3 * dim_w_ * dim_h_);
        l_it.reset();
        // do inverse mapping of the positions
        for(int i =0; i < dim_w_*dim_h_; i++ ){
          // inverse position
          int l_pos = l_it++;
          // inverse mapping
          result_[3*i] = input_data[3 * (l_pos) + 0];
          result_[3*i + 1] = input_data[3 * (l_pos) + 1];
          result_[3*i + 2] = input_data[3 * (l_pos) + 2];
        }
        return result_;
      }

      //! retrieve the results vector
      std::vector<cTypeOut> &getResult(){
        return result_;
      };

    protected:
      //! dimension of image grid we are mapping to
      int dim_w_,dim_h_;
      //! results vector
      std::vector<cTypeOut> result_;
      // iterator to iterate over the grid
      SnakeGridIterator l_it;
    };
  }
}
#endif//SNAKE_GRID_MAPPING_H
