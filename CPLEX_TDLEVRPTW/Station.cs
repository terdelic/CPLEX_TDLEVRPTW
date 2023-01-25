using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CPLEX_TDTSPTW
{
    //Class for CSs that inherits User class
    public class Station : User
    {
        public int _stationId;
        //Constructor
        public Station(int userId, int stId, double x, double y, double etw, double ltw, double stw, Params p) : base(userId, x, y, 0, etw, ltw, stw, UserType.Station, p)
        {
            this._stationId = stId;
        }
    }

}
