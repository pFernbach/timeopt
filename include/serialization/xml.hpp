
#ifndef __hpp_serialization_xml_hpp__
#define __hpp_serialization_xml_hpp__

#include <iostream>

namespace hpp
{
  namespace serialization
  {
    template<class C>
    struct serialize
    {
      template<class Archive>
      static operator(Archive & ar, C & c, const unsigned int version);
    };
    
  }
  
}

#endif // ifndef __hpp_serialization_xml_hpp__
