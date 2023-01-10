using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CPLEX_TDTSPTW
{
    public static class Misc
    {
        public static void errOut(string str)
        {
            //System.Diagnostics.Debug.WriteLine(str);
            Console.WriteLine(str);
            Console.ReadKey();
        }

        public static bool EqualDoubleValues(double val1, double val2, double precision)
        {
            if (Math.Abs(val1 - val2) <= precision)
            {
                return true;
            }
            return false;
        }

        //public static int roundToNearest(double val)
        //{
        //    return Convert.ToInt32(Math.Round(val));
        //}

    }
}
