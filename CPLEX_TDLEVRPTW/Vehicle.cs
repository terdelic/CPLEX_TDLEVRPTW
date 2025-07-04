using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CPLEX_TDTSPTW
{
    //Class for storing vehicle attributes
    public class Vehicle
    {
        //Attributes
        public Params p;
        public Solution s;
        public List<User> route;
        public int vehID;
        public double vehicleRestLoadCap;
        public double vehicleRestBatCap;
        public double vehicleTotalTime;
        public double vehicleTravelTime;
        public double vehicleDist;
        public double vehicleEnergy;
        public double vehicleRechargingTime;
        public int vehicleNumOfRecharges;
        public double vehicleRechargeAmount;
        //Constructor
        public Vehicle(Params p, int vehID, Solution s)
        {
            this.p = p;
            this.s = s;
            this.vehID = vehID;
            route = new List<User>();
            //New instance of depot set at the begining
            Customer newDepot = new Customer((Customer)p.depot);
            newDepot.posInRoute = route.Count;
            newDepot.isDepot = true;
            newDepot.restLoadCap = p.loadCap;
            newDepot.restBatCapAtArrival = p.batCap;
            newDepot.restBatCapAtDeparture = p.batCap;
            route.Add(newDepot);
            reset();
        }
        public void reset()
        {
            vehicleRestLoadCap = p.loadCap;
            vehicleRestBatCap = p.batCap;
            vehicleTotalTime = 0;
            vehicleTravelTime = 0;
            vehicleDist = 0;
            vehicleEnergy = 0;
            vehicleRechargingTime = 0;
            vehicleNumOfRecharges = 0;
            vehicleRechargeAmount = 0;
        }
        //Method to add depot at the end
        public void AddDepotToEnd()
        {
            //New instacne of depot added at the end
            Customer newDepot = new Customer((Customer)p.depot);
            newDepot.isDepot = true;
            newDepot.posInRoute = route.Count;
            route.Add(newDepot);
        }
        //Method to update all vehicle variables
        public void updateVehicle()
        {
            //Forward looping
            User uBefPomFor = route[0];
            User uCurPomFor = null;
            //Reset values
            vehicleTotalTime = 0;
            vehicleTravelTime = 0;
            vehicleDist = 0;
            vehicleEnergy = 0;
            vehicleRechargingTime = 0;
            vehicleNumOfRecharges = 0;
            vehicleRechargeAmount = 0;
            vehicleRestLoadCap = p.loadCap;
            vehicleRestBatCap = p.batCap;
            //User before departure time
            uBefPomFor.departureTime = uBefPomFor.etw + uBefPomFor.serviceTime;
            for (int uiFor = 1; uiFor < this.route.Count; uiFor++)
            {
                //Compute values for current user based on values from previous user
                uCurPomFor = route[uiFor];
                uCurPomFor.posInRoute = uiFor;
                uCurPomFor.vehicleID = this.vehID;
                //Arrival distance
                vehicleDist = vehicleDist + p.dist(uBefPomFor, uCurPomFor, uBefPomFor.departureTime);
                uCurPomFor.arrivalDistance = vehicleDist;
                //Travel time
                vehicleTravelTime += p.LinTime(uBefPomFor, uCurPomFor, uBefPomFor.departureTime);
                //Arrival time at user
                uCurPomFor.arrivalTime = uBefPomFor.departureTime + p.LinTime(uBefPomFor, uCurPomFor, uBefPomFor.departureTime);
                //Begin time of service at user
                uCurPomFor.beginTime = Math.Max(uCurPomFor.arrivalTime, uCurPomFor.etw);
                //Current value of total time
                vehicleTotalTime = uCurPomFor.beginTime;
                //Check late time window
                if (uCurPomFor.beginTime - uCurPomFor.ltw > p.doublePrecision)
                {
                    Misc.errOut("Arrival after late time window! Veh ID:" + vehID + "Position:" + uiFor + "!");
                }
                //Update rest load capacity
                vehicleRestLoadCap -= uCurPomFor.demand;
                uCurPomFor.restLoadCap = vehicleRestLoadCap;
                if (vehicleRestLoadCap < -p.doublePrecision)
                {
                    Misc.errOut("Load capacity violated! Veh ID:" + vehID + "Position:" + uiFor + "!");
                }
                //Update rest batter capacity at arrival and departure
                vehicleRestBatCap = vehicleRestBatCap - p.ener(uBefPomFor, uCurPomFor, uBefPomFor.departureTime);
                uCurPomFor.restBatCapAtArrival = vehicleRestBatCap;
                uCurPomFor.restBatCapAtDeparture = vehicleRestBatCap;
                if (vehicleRestBatCap < -p.doublePrecision)
                {
                    Misc.errOut("Battery capacity violated! Veh ID:" + vehID + "Position:" + uiFor + "!");
                }
                vehicleEnergy += p.ener(uBefPomFor, uCurPomFor, uBefPomFor.departureTime);
                uCurPomFor.energyConsumed = vehicleEnergy;
                //Assume the departure time (if its is CS it will updated)
                uCurPomFor.departureTime = uCurPomFor.beginTime + uCurPomFor.serviceTime;
                if (uCurPomFor.isStation())
                {
                    //Compte recharge time to full capacity
                    double timeCharge = p.refuelRate * (p.batCap - vehicleRestBatCap);
                    vehicleRechargeAmount += p.batCap - vehicleRestBatCap;
                    if (timeCharge - p.refuelRate * p.batCap > p.doublePrecision)
                    {
                        Misc.errOut("Punjenje duže od maksimalnog vremena!");
                    }
                    uCurPomFor.serviceTime=timeCharge;
                    vehicleRechargingTime += timeCharge;
                    vehicleNumOfRecharges++;
                    //Reset the rest battery capacity
                    vehicleRestBatCap = p.batCap;
                    //Set departure capacity at the fullest
                    uCurPomFor.restBatCapAtDeparture = p.batCap;
                    //Update departure time
                    uCurPomFor.departureTime = uCurPomFor.beginTime + timeCharge;
                }
                uBefPomFor = uCurPomFor;
            }
        }
    }
}

