using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CPLEX_TDTSPTW
{
    public class Station : User
    {
        public int _stationId;
        public double rechargedAmountInTime;
        public double rechargedAmount;

        public Station(int userId, int stId, double x, double y, double etw, double ltw, double stw, Params p) : base(userId, x, y, 0, etw, ltw, stw, UserType.Station, p)
        {
            this._stationId = stId;
        }
    }

}
