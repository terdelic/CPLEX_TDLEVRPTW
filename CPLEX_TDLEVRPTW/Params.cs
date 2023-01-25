using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CPLEX_TDTSPTW
{
    //Enum for minimization type
    public enum MinimizationType
    {
        Distance,
        TotalTime,
        TravelTime,
        Energy
    }
    //Enum for user type
    public enum UserType
    {
        Customer,
        Station
    };

    //Class for explicit form of linear line: y=slope*x+section
    public class LinEq
    {
        public double Slope; //Coefficient
        public double Section; //Rest
    }
    /* Class Params contains all the input data */
    public partial class Params
    {
        //Variables used when reading the input data
        public List<Tuple<string, string>> specificProblems; //List of specific problems that want to be solved
        public string instancesFile; //Path to instance files
        public string[] observedProblemType; // Problem type that wants to be solved: R1, RC2, C1 ...
        public string[] observedNumCust; //Number of customers: 5, 10 or 15
        //Enum variable for sotring minimization type: Distance, Travel Time etc...
        public MinimizationType minimizationType;
        //List for customers and CSs
        public List<Customer> customers;
        public List<Station> stations;
        //Users list contains both customers and CSs (not depot)
        public List<User> users;
        //Vechile battery capacity
        public double batCap;
        //Vechile load capacity
        public double loadCap;
        //Vechile linear consumption rate per distance traveled
        public double consRate;
        //Linear refuel rate per capacity needed to be recharged
        public double refuelRate;

        //Discretizied time buckets and speed in those time buckets
        public double[] timeBuckets;
        private double[] speedsInTimeBuckets;

        //This variable contains the FIgliozzi travel time coefficents type
        // A1-A3,..,D1-D3 and VRPTW
        public string travelTimeCompType;
        //Timlimt for cplex
        public int timeLimitCplex;
        //Memory limit for cplex
        public int memoryLimitCplex;
        //Number of virtual CSs in the MILP program
        public int numMultiCSVert;
        //Double precision used for comparison of floating point numbers
        public double doublePrecision;
        //Varible used to indicate wether the minimization of vehicles is neded or not
        public int knownVehNumberCPLEX;

        //Variables used when writng the output file
        public string outputFileName;
        //Name of the problem instance
        public string instanceName;
        public string orgSolomonName;
        public string orgNumCust;
        //Best known values for number of vehicles and cost reported by Schneider (2014) for EVRPTW
        public int BKSVehNum;
        public double BKSCost;

        //Matrices for arc costs: distance, travel time, energy
        private double[,,] distMat;
        private LinEq[,,] timeMat;
        private double[,,] energyMat;
        //Matrix for storing which arcs are infeasible (as expalined in the article, time-dependent case is not needed here)
        private bool[,] infeasibleArcs;
        //Minimum number of vehicles according to CVRP
        public int theoryMinNumVehicle;
        //Instance of depot and CS at depot
        public User depot;
        public Station CSAtDepot;
        //Bool variable to see if time buckets are of equal length
        private bool timeBucketEqual;
        //Length of equal time bucket
        private double timeBucketEqualLength;

        //Constructor
        public Params(string cfgFilePath)
        {
            //Load part of params from cfg.txt file
            try
            {
                StreamReader ifs = new StreamReader(cfgFilePath);
                //File conatining the list of all instances to be solved
                instancesFile = ifs.ReadLine().Split(':')[1].Trim();
                //Name of the output file
                outputFileName = ifs.ReadLine().Split(':')[1].Trim();
                //Type of minimization
                string strMinimizationType = ifs.ReadLine().Split(':')[1].Trim();
                minimizationType = GetMinimizationType(strMinimizationType);
                //If there is anyting written in the line specific problems, than parse them, add them to list specificProblems
                //and solve only this instances
                //Example: Specific problem instance:  C101-10
                string[] specProblems = (ifs.ReadLine().Split(':')[1].Trim()).Split(',');
                specificProblems = new List<Tuple<string, string>>();
                foreach (string specProblem in specProblems)
                {
                    if (string.IsNullOrEmpty(specProblem.Trim()))
                    {
                        continue;
                    }
                    string[] split = specProblem.Split('-');
                    specificProblems.Add(new Tuple<string, string>(split[0].Trim(), split[1].Trim()));
                }
                //If there are no specific problems, load observed number of customers and instance types
                //Example:
                //Observe problem type: C1,R1
                //Number of customers: 5,10
                if (specificProblems.Count == 0)
                {
                    observedProblemType = ifs.ReadLine().Split(':')[1].Trim().Split(',');
                    observedNumCust = ifs.ReadLine().Split(':')[1].Trim().Split(',');
                }
                else
                {
                    //Skip these two line if there are some specific problems loaded
                    ifs.ReadLine();
                    ifs.ReadLine();
                }
                //Figliozzi travel time coefficients
                travelTimeCompType = ifs.ReadLine().Split(':')[1].Trim();
                //Time limit for CPLEX
                timeLimitCplex = Convert.ToInt32(ifs.ReadLine().Split(':')[1].Trim());
                //Number of replication of CSs
                numMultiCSVert = Convert.ToInt32(ifs.ReadLine().Split(':')[1].Trim());
                //Floating point precision
                doublePrecision = Convert.ToDouble(ifs.ReadLine().Split(':')[1].Trim());
                //If knownVehNumberCPLEX is -1, the vehicle number minimization is performed, otherwise it is skipped and
                // secondary objective with knownVehNumberCPLEX vehicles is solved
                knownVehNumberCPLEX = Convert.ToInt32(ifs.ReadLine().Split(':')[1].Trim());
                //Memory limit for CPLEX in MB
                memoryLimitCplex = Convert.ToInt32(ifs.ReadLine().Split(':')[1].Trim());
                ifs.Close();
            }
            catch (Exception ex)
            {
                Misc.errOut(ex.Message);
            }
        }

        //Get enum minimization type from input string
        public MinimizationType GetMinimizationType(string minimizationType)
        {
            switch (minimizationType)
            {
                case "Distance":
                    return MinimizationType.Distance;
                case "TotalTime":
                    return MinimizationType.TotalTime;
                case "TravelTime":
                    return MinimizationType.TravelTime;
                case "Energy":
                    return MinimizationType.Energy;
                default:
                    Misc.errOut("Unknown minization type! Returning distance!");
                    return MinimizationType.Distance; ;
            }
        }

        //Method returns true if arc u1->u2 is infeasible
        public bool infeas(User u1, User u2)
        {
            return this.infeasibleArcs[u1._userID, u2._userID];
        }
        //Method returns Eucledian distance for arc u1->u2
        public double EucledianDistance(User u1, User u2)
        {
            if (u1 == u2)
            {
                Misc.errOut("Same u1 and u2 users in Eucledian distance!");
            }
            double value = Math.Sqrt(Math.Pow(u1.x - u2.x, 2) + Math.Pow(u1.y - u2.y, 2));
            return value;
        }

        //Loading specific instance params for selected instances
        public void loadInstanceSolomonEVRP(string numCust, string instanceNameArg, string dirConfData)
        {
            //Create path to instance file
            orgNumCust = numCust;
            orgSolomonName = instanceNameArg;
            instanceName = instanceNameArg + "C" + numCust;
            string instancePath = dirConfData + "\\evrptw_instances\\Cplex" + numCust + "er" + "\\" + instanceNameArg + "C" +
                           numCust + ".txt";

            try
            {
                StreamReader ifs = new StreamReader(instancePath);
                ifs.ReadLine(); //Skip HEADER
                //Initilaize lists
                users = new List<User>();
                customers = new List<Customer>();
                stations = new List<Station>();

                string[] splitLine;
                string line;
                //Varibale for sum of all demands
                double sumDemand = 0;
                while (true)
                {
                    line = ifs.ReadLine().Trim().Replace('.', ',');
                    splitLine = line.Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
                    //If there are no more lines, stop
                    if (splitLine.Length < 2)
                    {
                        break;
                    }
                    //Get the user atrributes
                    string name = splitLine[0];
                    string type = splitLine[1];
                    double x = Convert.ToDouble(splitLine[2]);
                    double y = Convert.ToDouble(splitLine[3]);
                    double demand = Convert.ToDouble(splitLine[4]);
                    double etw = Convert.ToDouble(splitLine[5]);
                    double ltw = Convert.ToDouble(splitLine[6]);
                    double stw = Convert.ToDouble(splitLine[7]);
                    //Increase demand
                    sumDemand += demand;
                    //depending on the user type, create an appropriate instance
                    if (type == "d")
                    {
                        Customer c = new Customer(users.Count, customers.Count, x, y, demand, etw, ltw, stw, this);
                        c.isDepot = true;
                        this.depot = c;
                        users.Add(c);
                    }
                    else if (type == "f")
                    {
                        Station s = new Station(users.Count, stations.Count, x, y, etw, ltw, stw, this);
                        users.Add(s);
                        stations.Add(s);
                        if (Misc.EqualDoubleValues(depot.x, x, doublePrecision) && Misc.EqualDoubleValues(depot.y, y, doublePrecision))
                        {
                            CSAtDepot = s;
                        }
                    }
                    else if (type == "c")
                    {
                        Customer c = new Customer(users.Count, customers.Count, x, y, demand, etw, ltw, stw, this);
                        customers.Add(c);
                        users.Add(c);
                    }
                    else
                    {
                        Misc.errOut("Uknow user type!");
                    }
                }
                //Load battery capacity for problem
                splitLine = ifs.ReadLine().Trim().Replace('.', ',').Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
                batCap = Convert.ToDouble(splitLine[splitLine.Length - 1].Replace('/', ' ').Trim());
                //Load load (cargo) capacity for problem
                splitLine = ifs.ReadLine().Trim().Replace('.', ',').Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
                loadCap = Convert.ToDouble(splitLine[splitLine.Length - 1].Replace('/', ' ').Trim());
                //Load consumption rate for problem
                splitLine = ifs.ReadLine().Trim().Replace('.', ',').Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
                consRate = Convert.ToDouble(splitLine[splitLine.Length - 1].Replace('/', ' ').Trim());
                //Load refuel rate for problem
                splitLine = ifs.ReadLine().Trim().Replace('.', ',').Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
                refuelRate = Convert.ToDouble(splitLine[splitLine.Length - 1].Replace('/', ' ').Trim());
                ifs.Close();

                //Theory minimum number of vehicles in CVRP
                theoryMinNumVehicle = Convert.ToInt32(Math.Ceiling(sumDemand / loadCap));
            }
            catch (Exception ex)
            {
                Misc.errOut(ex.Message);
            }

            //Load speeds and travel times from Filiozzi files
            try
            {
                //Open stream to Figliozzi file
                StreamReader ifsFig = new StreamReader(dirConfData + "FigllioziCoeffs.txt");
                ifsFig.ReadLine(); //HEADER
                ifsFig.ReadLine(); //HEADER
                string line = ifsFig.ReadLine().Trim().Replace('.', ',');
                string[] splitLine = line.Split(new char[] { ';' }, StringSplitOptions.RemoveEmptyEntries);
                //Load time buckets discrete time points
                timeBuckets = new double[splitLine.Length];
                timeBucketEqual = true;
                double previousTB = 0;
                double previousDiffTB = 0;
                for (int i = 0; i < splitLine.Length; i++)
                {
                    timeBuckets[i] = Convert.ToDouble(splitLine[i]) * this.depot.ltw;
                    timeBucketEqualLength = timeBuckets[i] - previousTB;
                    if (timeBucketEqual &&
                        !Misc.EqualDoubleValues(previousTB, 0, doublePrecision) &&
                        !Misc.EqualDoubleValues(timeBucketEqualLength, previousDiffTB, doublePrecision))
                    {
                        timeBucketEqual = false;
                    }
                    previousTB = timeBuckets[i];
                    previousDiffTB = timeBucketEqualLength;
                }
                //Find speed coeffs in time buckets for loaded travel tiem type, i.e. A1 
                while (!ifsFig.EndOfStream)
                {
                    line = ifsFig.ReadLine().Trim();
                    if (!line.Contains(travelTimeCompType))
                    {
                        continue;
                    }
                    line = ifsFig.ReadLine().Trim().Replace('.', ',');
                    splitLine = line.Split(new char[] { ';' }, StringSplitOptions.RemoveEmptyEntries);
                    speedsInTimeBuckets = new double[timeBuckets.Length];
                    if (timeBuckets.Length != splitLine.Length)
                    {
                        Misc.errOut("Wrong number of time buckets!");
                    }
                    for (int i = 0; i < splitLine.Length; i++)
                    {
                        speedsInTimeBuckets[i] = Convert.ToDouble(splitLine[i]);
                    }
                }
                ifsFig.Close();
            }
            catch (Exception ex)
            {
                Misc.errOut(ex.Message);
            }
            //Load matrices
            generateMatrix();
            //Set infeasible arcs
            setInfeasibleArcs();
        }

        //Method to get the time bucket k interval for set time value
        public int getTimeIntervalIndex(double timeParam)
        {
            //If time buckest are of equal length than the faster way can be used to obtain the time bucket index, otherwise the loop is used
            if (timeBucketEqual)
            {
                int k = Convert.ToInt32(Math.Floor(timeParam / timeBucketEqualLength));
                if (k < 0)
                {
                    return 0;
                }
                else if (k > timeBuckets.Length - 1)
                {
                    return timeBuckets.Length - 1;
                }
                else
                {
                    return k;
                }
            }
            else
            {
                for (int k = 0; k < timeBuckets.Length; k++)
                {
                    if (timeParam - timeBuckets[k] < 0) //Has to be lower than zero (throws an error on distance check when doublePRecision is used)
                    {
                        return k;
                    }
                }
                return timeBuckets.Length - 1;
            }
        }

        //Function to get distance of an arc u1->u2 in specific time period k
        public double dist(User u1, User u2, int k)
        {
            return distMat[u1._userID, u2._userID, k];
        }

        //Function to get distance of an arc u1->u2  depending on the departure time
        public double dist(User u1, User u2, double departureTimeU1)
        {
            int k = getTimeIntervalIndex(departureTimeU1);
            return distMat[u1._userID, u2._userID, k];
        }

        //Function to get linear time of an arc u1->u2  depending on the departure time
        public double LinTime(User u1, User u2, double departureTimeU1)
        {
            int k = getTimeIntervalIndex(departureTimeU1);
            LinEq eq = timeMat[u1._userID, u2._userID, k];
            return eq.Slope * departureTimeU1 + eq.Section;
        }

        //Function to get slope of linear time of an arc u1->u2  in specific time period k
        public double getSlopeFromLinTime(User u1, User u2, int k)
        {
            return timeMat[u1._userID, u2._userID, k].Slope;
        }

        //Function to get section of linear time of an arc u1->u2 in specific time period k
        public double getSectionFromLinTime(User u1, User u2, int k)
        {
            return timeMat[u1._userID, u2._userID, k].Section;
        }

        //Function to get energyof an arc u1->u2  depending on the departure time
        public double ener(User u1, User u2, double departureTimeU1)
        {
            int k = this.getTimeIntervalIndex(departureTimeU1);
            return this.energyMat[u1._userID, u2._userID, k];
        }

        //Function to get energy of an arc u1->u2  depending on the time period k
        public double ener(User u1, User u2, int k)
        {
            return this.energyMat[u1._userID, u2._userID, k];
        }

        //Figliozzi travel time function (look in the article) -- assumes that distance does not change in time buckets
        public double FigTime(User u1, User u2, double departureTimeU1)
        {
            //Get the current index of time period
            int k = getTimeIntervalIndex(departureTimeU1);
            double totalDistance = dist(u1, u2, k);
            //Compute the arrival time at user u2 (distance does not change)
            double finalTime = departureTimeU1 + totalDistance / speedsInTimeBuckets[k];
            //Get the next index of time period at the arrival time
            int kNext = getTimeIntervalIndex(finalTime);
            double departureTime = departureTimeU1;
            //If k and kNext are not neighbors, first go the the next neighboring period
            kNext = kNext > k + 1 ? k + 1 : kNext;
            while (k != kNext)
            {
                //Compute time and distance traveled in time bucket k
                double timeInKBucket = timeBuckets[k] - departureTime;
                totalDistance = totalDistance - timeInKBucket * speedsInTimeBuckets[k];
                if (totalDistance < -doublePrecision)
                {
                    Misc.errOut("Distance is lower than zero!");
                }
                //Set the departure time at end of time bucket k, and compute the time traveled in time bucket kNext
                departureTime = timeBuckets[k];
                finalTime = departureTime + totalDistance / speedsInTimeBuckets[kNext];
                k = kNext;
                kNext = getTimeIntervalIndex(finalTime);
                kNext = kNext > k + 1 ? k + 1 : kNext;
            }
            //The total travel time is difference between the final arrival time and departure time
            return finalTime - departureTimeU1;
        }

        /*The function that linearizes the travel time into function y=kx+l (line through two points)
        * x...departure times (dt1,dt2)
        * y...travel time (tt1,tt2)
         */
        public LinEq compLinTTEquation(double dt1, double tt1, double dt2, double tt2)
        {
            //Careful to not divide by zero
            if (Math.Abs(dt1 - dt2) < doublePrecision)
            {
                Misc.errOut("There is no devision by 0!");
            }
            double slope = (tt2 - tt1) / (dt2 - dt1);
            //This is the resolution problem, for example
            // when all speed coeffs are 1, the travel time is constant function and should be zero but due to he double resolution it is not
            if (Math.Abs(slope) < doublePrecision)
            {
                slope = 0;
            }
            double section = tt1 - slope * dt1;
            return new LinEq() { Section = section, Slope = slope };
        }

        //Generate time, distance and energy matrices
        public void generateMatrix()
        {
            //3-D matrices for distances, travel time and energy 
            distMat = new Double[users.Count, users.Count, timeBuckets.Length];
            timeMat = new LinEq[users.Count, users.Count, timeBuckets.Length];
            energyMat = new double[users.Count, users.Count, timeBuckets.Length];
            //The commented code was used to plot figure in the article -> will be removed
            //StreamWriter sw = new StreamWriter("travelTime.txt");
            //StreamWriter desc = new StreamWriter("desc.txt");
            for (int i = 0; i < users.Count; i++)
            {
                for (int j = 0; j < users.Count; j++)
                {
                    //The i==j is forbbiden, and the distance is set to 0
                    if (i != j)
                    {
                        User userI = users[i];
                        User userJ = users[j];
                        //First start departure time is depot early time window
                        double startDepartureTime = depot.etw;
                        for (int k = 0; k < timeBuckets.Length; k++)
                        {
                            //End departure time is the first time bucket dicretizied time,
                            //but reduced by double precision, as interval is [T_k,T_{k+1}>
                            double endDepartureTime = timeBuckets[k] - doublePrecision;
                            //Distance is computed as eucledian distance
                            //It is currently considered that distance does not change depending on the departure time
                            //IMPORTNAT to change if distance will be time dependent
                            distMat[i, j, k] = EucledianDistance(users[i], users[j]);
                            //Travel time matrices that contain the slope and section for each time period
                            //The travel time is computed using Figliozzi function that actually linearizes
                            // the travel time itslef (but not the travel time bewteen the different time periods - 
                            // that why we linearize it)
                            timeMat[i, j, k] = compLinTTEquation(startDepartureTime, FigTime(userI, userJ, startDepartureTime),
                                endDepartureTime, FigTime(userI, userJ, endDepartureTime));
                            //Energy matrix
                            energyMat[i, j, k] = consRate * distMat[i, j, k];
                            //Update start departure time
                            startDepartureTime = timeBuckets[k];
                        }

                        //double startTime = depot.etw;
                        //double resolution = 0.1;
                        //double currentTime = startTime;
                        //string line1 = "", line2 = "", line3 = "",line4="", line5="("+i+"->"+j+")";
                        //while (currentTime <= depot.ltw)
                        //{
                        //    double FTime = FigTime(userI, userJ, currentTime);

                        //    line1+=currentTime;
                        //    line2 += FTime;
                        //    line3 += TimeUnformSpeed(userI,userJ,currentTime);
                        //    line4+=LinTime(userI, userJ, currentTime);


                        //    if (currentTime < depot.ltw)
                        //    {
                        //        line1 += ";";
                        //        line2 += ";";
                        //        line3 += ";";
                        //        line4 += ";";

                        //    }
                        //    currentTime += resolution;
                        //}
                        //sw.WriteLine(line1.Replace(',','.'));
                        //sw.WriteLine(line2.Replace(',', '.'));
                        //sw.WriteLine(line3.Replace(',', '.'));
                        //sw.WriteLine(line4.Replace(',', '.'));
                    }
                }
            }
            //sw.Close();
        }

        /*
         * This function returns the lower and upper dicretizied time point values (as a double tuple) for specific time period k 
         */
        public Tuple<double, double> getBoundaries(int k)
        {
            Tuple<double, double> t;
            if (k < 0 || k >= timeBuckets.Length)
            {
                Misc.errOut("Wrong k value!");
            }
            if (k == 0)
            {
                t = new Tuple<double, double>(depot.etw, timeBuckets[k]);
            }
            else
            {
                t = new Tuple<double, double>(timeBuckets[k - 1], timeBuckets[k]);
            }
            return t;
        }

        //Method to set infeasible arcs
        private void setInfeasibleArcs()
        {
            //All arcs are in the begining feasible: false means feasible, true infeasible
            infeasibleArcs = new bool[users.Count, users.Count];
            for (int i = 0; i < users.Count; i++)
            {
                for (int j = 0; j < users.Count; j++)
                {
                    User userI = users[i];
                    User userJ = users[j];
                    if (i == j) // Obvious condition
                    {
                        infeasibleArcs[i, j] = true;
                        continue;
                    }
                    /*IMPORTANT - CAREFUL
                     * This is related only to the benchamrk instance and
                    have to be very careful with that - as user with ID =1 is CS located at the depot
                    there is no point in going from depot to that CS (the same goes in the other way around)
                    */
                    if (CSAtDepot != null && userI.isDepot && userJ == CSAtDepot)
                    {
                        infeasibleArcs[i, j] = true;
                        continue;
                    }
                    if (CSAtDepot != null && userJ.isDepot && userI == CSAtDepot)
                    {
                        infeasibleArcs[i, j] = true;
                        continue;
                    }
                    //Capacity violation - does not depend on time bucket k
                    //tex:$q_i+q_j > C \wedge i \in V_0 \cup F', j \in V_{ED} \cup F'$
                    if (userI.demand + userJ.demand - loadCap > doublePrecision)
                    {
                        infeasibleArcs[i, j] = true;
                        continue;
                    }
                    //time window violation - does not depend on the time bucket k
                    //tex: $e_i+s_i+t_{ij}^k>l_j\wedge i \in V_0 \cup F', j \in V \cup F'$
                    if (userI.etw + userI.serviceTime + LinTime(userI, userJ, userI.etw + userI.serviceTime) - userJ.ltw > doublePrecision)
                    {
                        infeasibleArcs[i, j] = true;
                        continue;
                    }
                    ////Depot late time windows - does not depend on the time bucket k
                    //tex:  $max(e_i+s_i+t_{ij}^k,e_j)+s_j+t_{j,N+1}^k>l_0 \wedge i \in V_0 \cup F', j \in V \cup F'$
                    if (!(userJ.isDepot))
                    {
                        double beginTimeAtUserJ = userI.compBeginTm(userI, userJ, userI.etw, userI.serviceTime);
                        double arrivalTimeAtDepot =
                            userJ.compArrivalTm(userJ, depot, beginTimeAtUserJ, userJ.serviceTime);
                        if (arrivalTimeAtDepot - depot.ltw > this.doublePrecision)
                        {
                            infeasibleArcs[i, j] = true;
                            continue;
                        }
                    }
                    // Violation of battery capacity - Sciffer LRPIF 2018
                    // Currently, the energy does not changes dependent on the time period k,
                    // but we still check if in any of the time windows k the energy is feasible, we say it is feaasible in all
                    //
                    //tex:  $e_{ij}^k>Q \wedge i \in V_0 \cup F', j \in V_{ED} \cup F'$
                    bool infeasibleEnergy = true;
                    double departTime = 0;
                    for (int k = 0; k < timeBuckets.Length; k++)
                    {
                        if (batCap - ener(userI, userJ, departTime) > doublePrecision)
                        {
                            infeasibleEnergy = false;
                            break;
                        }
                        departTime = timeBuckets[k];
                    }
                    if (infeasibleEnergy)
                    {
                        infeasibleArcs[i, j] = true;
                        continue;
                    }
                    /*
                     * If vechile goes from user I to user J, and imidiately before I it visits the CS, and also imidiately after J
                     * also visits a CS, and there is no combination of CSs that make this feasible, then the arc is infeasible 
                     * We scale that, as in previous case, that in all time bucetks the energy has to be infeaasible for arc to be infeasbile
                    
                     */
                    if (!(userI.isDepot) && !(userJ.isDepot) && !(userI.isStation()) && !(userJ.isStation()))
                    {
                        departTime = 0;
                        infeasibleEnergy = true;
                        for (int k = 0; k < timeBuckets.Length; k++)
                        {
                            List<User> pomStations = new List<User>(stations);
                            for (int IS = 0; IS < stations.Count(); IS++)
                            {
                                for (int JS = 0; JS < stations.Count(); JS++)
                                {
                                    User stI = stations[IS];
                                    User stJ = stations[JS];
                                    if (ener(stI, userI, departTime) + ener(userI, userJ, departTime) + ener(userJ, stJ, departTime) <= batCap)
                                    {
                                        infeasibleEnergy = false;
                                        break;
                                    }
                                }
                                if (infeasibleEnergy == false)
                                {
                                    break;
                                }
                            }
                            if (infeasibleEnergy == false)
                            {
                                break;
                            }
                            departTime = timeBuckets[k];
                        }
                        if (infeasibleEnergy == true)
                        {
                            infeasibleArcs[i, j] = true;
                            continue;
                        }
                    }
                }
            }
        }
    }
}

