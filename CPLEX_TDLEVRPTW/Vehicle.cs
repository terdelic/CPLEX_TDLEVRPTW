using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CPLEX_TDTSPTW
{
    public class Vehicle
    {

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

        public Vehicle(Params p, int vehID, Solution s)
        {
            this.p = p;
            this.s = s;
            this.vehID = vehID;
            route = new List<User>();
            //Bitno je da se stvara nova instanca skladišta
            //MOžda i netreba ove sve varijable postavljati, možda samo is depot na true
            Customer newDepot = new Customer((Customer)p.depot);
            newDepot.posInRoute = route.Count;
            newDepot.isDepot = true;
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

        public void AddDepotToEnd()
        {
            //Bitno je da je nova instanca depota
            Customer newDepot = new Customer((Customer)p.depot);
            newDepot.isDepot = true;
            newDepot.posInRoute = route.Count;
            route.Add(newDepot);
        }
        //Function to update of vehicle variables
        public void updateVehicle()
        {
            //Forward looping
            User uBefPomFor = route[0];
            User uCurPomFor = null;

            vehicleTotalTime = 0;
            vehicleTravelTime = 0;
            vehicleDist = 0;
            vehicleEnergy = 0;
            vehicleRechargingTime = 0;
            vehicleNumOfRecharges = 0;
            vehicleRechargeAmount = 0;

            uBefPomFor.departureTime= uBefPomFor.etw + uBefPomFor.serviceTime;
            vehicleRestLoadCap = p.loadCap;
            vehicleRestBatCap = p.batCap;
            for (int uiFor = 1; uiFor < this.route.Count; uiFor++)
            {
                uCurPomFor = route[uiFor];
                uCurPomFor.posInRoute = uiFor;
                uCurPomFor.vehicleID = this.vehID;

                vehicleDist = vehicleDist + p.dist(uBefPomFor, uCurPomFor, uBefPomFor.departureTime);
                uCurPomFor.arrivalDistance = vehicleDist;
                vehicleTravelTime += p.LinTime(uBefPomFor, uCurPomFor, uBefPomFor.departureTime);
                uCurPomFor.arrivalTime = uBefPomFor.departureTime + p.LinTime(uBefPomFor, uCurPomFor, uBefPomFor.departureTime);
                uCurPomFor.beginTime = Math.Max(uCurPomFor.arrivalTime, uCurPomFor.etw);
                vehicleTotalTime = uCurPomFor.beginTime;

                if (uCurPomFor.beginTime - uCurPomFor.ltw > p.doublePrecision)
                {
                    Misc.errOut("Arrival after late time window! Veh ID:" + vehID + "Position:" + uiFor + "!");
                }

                vehicleRestLoadCap -= uCurPomFor.demand;
                uCurPomFor.restLoadCap = vehicleRestLoadCap;
                vehicleRestBatCap = vehicleRestBatCap - p.ener(uBefPomFor, uCurPomFor, uBefPomFor.departureTime);
                uCurPomFor.restBatCapAtArrival = vehicleRestBatCap;
                uCurPomFor.restBatCapAtDeparture = vehicleRestBatCap;
                if (vehicleRestLoadCap < -p.doublePrecision)
                {
                    Misc.errOut("Load capacity violated! Veh ID:" + vehID + "Position:" + uiFor + "!");
                }
                if (vehicleRestBatCap < -p.doublePrecision)
                {
                    Misc.errOut("Battery capacity violated! Veh ID:" + vehID + "Position:" + uiFor + "!");
                }

                vehicleEnergy += p.ener(uBefPomFor, uCurPomFor, uBefPomFor.departureTime);
                uCurPomFor.energyConsumed = vehicleEnergy;
                uCurPomFor.departureTime = uCurPomFor.beginTime + uCurPomFor.serviceTime;
                if (uCurPomFor.isStation())
                {
                    double timeCharge = p.refuelRate * (p.batCap - vehicleRestBatCap);
                    vehicleRechargeAmount += p.batCap - vehicleRestBatCap;
                    if (timeCharge - p.refuelRate * p.batCap > p.doublePrecision)
                    {
                        Misc.errOut("Punjenje duže od maksimalnog vremena!");
                    }
                    vehicleRechargingTime += timeCharge;
                    vehicleNumOfRecharges++;
                    vehicleRestBatCap = p.batCap;
                    uCurPomFor.restBatCapAtDeparture = p.batCap;
                    uCurPomFor.departureTime = uCurPomFor.beginTime + timeCharge;
                }
                uBefPomFor = uCurPomFor;
            }
        }
    }
}

