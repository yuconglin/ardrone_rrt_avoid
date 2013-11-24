#pragma once
//forward declaration
namespace user_types{
   struct ObsCollect;
   struct GeneralState;
}

namespace utils{

   bool CollectCheck(const user_types::GeneralState* st_pt, user_types::ObsCollect& obscollect);

};
