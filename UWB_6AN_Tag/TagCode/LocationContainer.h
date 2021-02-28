#include "RangingContainer.h"

class Location {
    public:
    int num_anchors = 3;
    int Initiator[2] = {0,0};     //Assume anchor A at (0,0)
    int Responder[num_anchors-1][2]; //Location of anchor B: Responder[0]; anchor C: Responder[1]

//    Ranging range[num_anchors-1]; //Struct used for computing 

   double delta_dist[num_anchors-1];    //Contains difference of distance: d_AB, d_AC
   double Loc[2];                  // Location of the tag

       
   void Calculate_location() 
   {
    // Location code here
    
    }

   
  void initialize() {
    for(int i=0;i<num_anchors-1;i++){
      delta_dist[i] = 0;
      }
    Loc = {0,0};
  }

  
};
