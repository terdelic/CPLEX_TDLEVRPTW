using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CPLEX_TDTSPTW
{
    public class User
    {
        public int _userID;
        public double x;
        public double y;
        public double demand;
        public double etw;
        public double ltw;
        public double serviceTime;

        //public double rechargeTime;
        public UserType type;
        public bool isDepot;
        public Params p;


        //public Vehicle vehicle;
        public int vehicleID;

        public int posInRoute;

        public double departureTime;
        public double arrivalDistance;
        public double arrivalTime;
        public double beginTime;
        public double restLoadCap;
        public double restBatCapAtArrival;
        public double restBatCapAtDeparture;
        public double energyConsumed;



        public User(int id, double x, double y, double demand, double etw, double ltw, double stw, UserType type, Params p)
        {
            this._userID = id;
            this.x = x;
            this.y = y;
            this.demand = demand;
            this.etw = etw;
            this.ltw = ltw;
            this.serviceTime = stw;
            this.type = type;
            this.p = p;

            this.isDepot = false;
            resetUserValues();
        }


        public virtual void resetUserValues()
        {
            //vehicle = null;
            vehicleID = -1;
            posInRoute = -1;

            departureTime = this.etw + this.serviceTime;
            arrivalDistance = 0;
            arrivalTime = 0;
            beginTime = this.etw;
            restLoadCap = 0;
            restBatCapAtArrival = 0;
            restBatCapAtDeparture = 0;
            energyConsumed = 0;

        }

        public bool isStation()
        {
            if (this.type == UserType.Station)
            {
                return true;
            }
            return false;
        }

        public bool isDepotOrStation()
        {
            if (this.type == UserType.Station || this.isDepot)
            {
                return true;
            }
            return false;
        }
        //Compute begin time at user
        public double compBeginTm(User userBef, User userCur, double befBeginTime, double befServiceTime)
        {
            double compArrivalTime = compArrivalTm(userBef, userCur, befBeginTime, befServiceTime);
            return Math.Max(compArrivalTime, userCur.etw);
        }
        //Compute arrival time at user
        public double compArrivalTm(User userBef, User userCur, double befBeginTime, double befServiceTime)
        {
            return befBeginTime + befServiceTime + p.LinTime(userBef, userCur, befBeginTime + befServiceTime);
        }
    }
}
