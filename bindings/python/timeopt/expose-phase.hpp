#ifndef __hpp_timeopt_python_expose_phase_hpp__
#define __hpp_timeopt_python_expose_phase_hpp__

#include "timeopt/bindings/python/timeopt/phase.hpp"

namespace timeopt
{
    namespace python
    {
        void exposePhaseInfo();
        inline void exposePhase()
        {
        exposePhaseInfo();
        }
        
    } // namespace python
} // namespace timeopt

#endif // ifndef __hpp_timeopt_python_expose_phase_hpp__
