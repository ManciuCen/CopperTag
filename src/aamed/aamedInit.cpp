// #include "tagDetector.h"
#include "aamed/AAMED.h"

// namespace CopperTag{
  double AAMED::get_T_edge_num(double theta_fsa, double &_T_min_minor)
  {
    _T_min_minor = 1 / (1 - cos(theta_fsa / 2));
    
    return _T_min_minor * 2 * theta_fsa;
  }

  void AAMED::set_parameters(double theta_fsa, double length_fsa, double T_val)
  {
    assert(theta_fsa >= 0 || theta_fsa <= CV_PI / 2);
    assert(length_fsa > 1);
    assert(T_val > 0);

    _theta_fsa = theta_fsa;
    _length_fsa = length_fsa;
    _T_val = T_val;

    _T_edge_num = get_T_edge_num(theta_fsa, _T_min_minor);
  }
// }