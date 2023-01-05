using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CPLEX_TDTSPTW
{
    public class Solution
    {
        public List<Vehicle> vehicles;
        //Variable used evaluate the solution
        public double solutionDist;
        public double solutionTotalTime;
        public double solutionTravelTime;
        public double solutionEnergy;
        public double solutionRechargingTime;
        public int solutionNumOfRecharges;
        public double solutionRechargeAmount;

        public Params p;
        public Solution(Params p)
        {
            this.p = p;
            vehicles = new List<Vehicle>();
            reset();
        }       
        public void reset()
        {         
            solutionDist = 0;
            solutionTotalTime = 0;
            solutionTravelTime = 0;
            solutionEnergy = 0;
            solutionRechargingTime = 0;
            solutionNumOfRecharges = 0;
            solutionRechargeAmount = 0;
        }
        //Add vehicle to list of vehicles and increase solution values
        public void addVehicle(Vehicle v)
        {
            vehicles.Add(v);
            solutionDist += v.vehicleDist;
            solutionTotalTime += v.vehicleTotalTime;
            solutionTravelTime += v.vehicleTravelTime;
            solutionEnergy += v.vehicleEnergy;
            solutionNumOfRecharges += v.vehicleNumOfRecharges;
            solutionRechargingTime += v.vehicleRechargingTime;
            solutionRechargeAmount += v.vehicleRechargeAmount;
        }

        //Convert solution to string used to write it into txt file, d is demilited
        public string getStrSolutionDetails(string d)
        {
            string line = d + this.solutionDist + d + this.solutionTravelTime + d + this.solutionTotalTime + d +
                this.solutionEnergy + d + this.solutionRechargingTime + d+this.solutionRechargeAmount+d + solutionNumOfRecharges+d;
            //To be able to reconstruct solution in the case if addtional values will have to be computed
            //I also added the complete configuration of solution (list of users) in the string
            foreach (Vehicle v in vehicles)
            {
                foreach (User u in v.route)
                {
                    line += u._userID;
                    if (v.route.Last() != u)
                    {
                        line += "|";
                    }
                }
                if (vehicles.Last() != v)
                {
                    line += "--";
                }
            }
            return line;
        }
    }
}
