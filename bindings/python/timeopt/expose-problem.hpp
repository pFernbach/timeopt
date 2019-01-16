#ifndef __timeopt_python_expose_problem_hpp__
#define __timeopt_python_expose_problem_hpp__

#include "timeopt/bindings/python/timeopt/problem.hpp"
#include "timeopt/bindings/python/timeopt/enum.hpp"
namespace timeopt
{
    namespace python
    {
        void exposeProblemInfo();
        void exposeTimeoptEnums();

        inline void exposeProblem()
        {
          exposeProblemInfo();
          exposeTimeoptEnums();
        }
        
    } // namespace python
} // namespace timeopt

#endif // ifndef __timeopt_python_expose_problem_hpp__
