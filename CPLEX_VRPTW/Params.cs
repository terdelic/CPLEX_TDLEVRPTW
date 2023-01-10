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
        public double slope; //Koeficijent
        public double section; //Odsjecak
    }
    /* Class Params contains all the input data */
    public partial class Params
    {
        //Variables used when reading the input data
        public List<Tuple<string, string>> specificProblems;
        public string instancesFile;
        public string[] observedProblemType;
        public string[] observedNumCust;

        public MinimizationType minimizationType;


        public List<Customer> customers;
        //Users list contains both customers and stations (not depot)
        public List<User> users;

        //Vechile load capacity
        public double loadCap;

        //Timlimt for cplex
        public int timeLimitCplex;
        //Memory limit for cplex
        public int memoryLimitCplex;
        //Double precision used for comparison of real numbers
        public double doublePrecision;

        //Variables used when writng the output file
        public string outputFileName;
        public string instanceName;
        public string orgSolomonName;
        public string orgNumCust;
        public int BKSVehNum;
        public double BKSCost;


        //Matrices for arc costs
        private double[,] distMat;
        private double[,] timeMat;
        private bool[,] infeasibleArcs;


        public User depot;

        //Varible used to indicate wether the minimization of vehicles is neded or not
        public int knownVehNumberCPLEX;
        public int theoryMinNumVehicle;

        public Params(string mainDir, string cfgFilePath)
        {
            //Load part of params from cfg.txt file
            try
            {
                StreamReader ifs = new StreamReader(cfgFilePath);
                instancesFile = ifs.ReadLine().Split(':')[1].Trim();
                outputFileName = ifs.ReadLine().Split(':')[1].Trim();
                string strMinimizationType = ifs.ReadLine().Split(':')[1].Trim();
                minimizationType = GetMinimizationType(strMinimizationType);
                //If there is anyting written in the line specific problems, than parse it and solve only this instance
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
                //If there are no specific problem load observed number of customers and instance types
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
                    ifs.ReadLine();
                    ifs.ReadLine();
                }
                timeLimitCplex = Convert.ToInt32(ifs.ReadLine().Split(':')[1].Trim());
                doublePrecision = Convert.ToDouble(ifs.ReadLine().Split(':')[1].Trim());
                knownVehNumberCPLEX = Convert.ToInt32(ifs.ReadLine().Split(':')[1].Trim());
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
                    Misc.errOut("Unknown minization type! Returning distance!");
                    return MinimizationType.Distance; ;
                case "TravelTime":
                    Misc.errOut("Unknown minization type! Returning distance!");
                    return MinimizationType.Distance; ;
                case "Energy":
                    Misc.errOut("Unknown minization type! Returning distance!");
                    return MinimizationType.Distance; ;
                default:
                    Misc.errOut("Unknown minization type! Returning distance!");
                    return MinimizationType.Distance; ;
            }
        }

        public bool infeas(User u1, User u2)
        {
            return this.infeasibleArcs[u1._userID, u2._userID];
        }

        public double EucledianDistance(User u1, User u2)
        {
            if (u1 == u2)
            {
                Misc.errOut("Same u1 and u2 users in Eucledian distance!");
            }
            double value = Math.Sqrt(Math.Pow(u1.x - u2.x, 2) + Math.Pow(u1.y - u2.y, 2));
            return value;
        }

        //Loading specific params for selected instances
        public void loadInstanceSolomonEVRP(string numCust, string instanceNameArg, string dirConfData)
        {
            orgNumCust = numCust;
            orgSolomonName = instanceNameArg;
            instanceName = instanceNameArg + "C" + numCust;
            string instancePath = dirConfData + "\\evrptw_instances\\Cplex" + numCust + "er" + "\\" + instanceNameArg + "C" +
                           numCust + ".txt";

            try
            {
                StreamReader ifs = new StreamReader(instancePath);
                ifs.ReadLine(); //Skip HEADER
                users = new List<User>();
                customers = new List<Customer>();

                string[] splitLine;
                string line;
                double sumDemand = 0;
                while (true)
                {
                    line = ifs.ReadLine().Trim().Replace('.', ',');
                    splitLine = line.Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
                    if (splitLine.Length < 2)
                    {
                        break;
                    }
                    string name = splitLine[0];
                    string type = splitLine[1];
                    double x = Convert.ToDouble(splitLine[2]);
                    double y = Convert.ToDouble(splitLine[3]);
                    //double demand = Convert.ToDouble(splitLine[4]);
                    //double etw = Convert.ToDouble(splitLine[5]);
                    //double ltw = Convert.ToDouble(splitLine[6]);
                    //double stw = Convert.ToDouble(splitLine[7]);

                    //FOR tsp
                    double demand = 0;
                    double etw = 0;
                    double ltw = 10000000;
                    double stw = 0;

                    sumDemand += demand;
                    if (type == "d")
                    {
                        Customer c = new Customer(users.Count, customers.Count, x, y, demand, etw, ltw, stw, this);
                        c.isDepot = true;
                        this.depot = c;
                        users.Add(c);
                    }
                    else if (type == "f")
                    {
                        continue;
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
                splitLine = ifs.ReadLine().Trim().Replace('.', ',').Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
                splitLine = ifs.ReadLine().Trim().Replace('.', ',').Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
                loadCap = Convert.ToDouble(splitLine[splitLine.Length - 1].Replace('/', ' ').Trim());
                //For tsp
                loadCap = 10000;
                splitLine = ifs.ReadLine().Trim().Replace('.', ',').Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
                splitLine = ifs.ReadLine().Trim().Replace('.', ',').Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
                ifs.Close();

                //Teoretski minimalan broj vozila kao da je CVRP
                theoryMinNumVehicle = Convert.ToInt32(Math.Ceiling(sumDemand / loadCap))==0?1: Convert.ToInt32(Math.Ceiling(sumDemand / loadCap));

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

        //Function to get distance between users in specific time period k
        public double dist(User u1, User u2)
        {
            return distMat[u1._userID, u2._userID];
        }
        //Function to get linear time between users depending on the departure time
        public double getTime(User u1, User u2)
        {
            return timeMat[u1._userID, u2._userID];
        }
        public void generateMatrix()
        {
            //3-D matrices for distances, travel time and energy 
            distMat = new double[users.Count, users.Count];
            timeMat = new double[users.Count, users.Count];
            for (int i = 0; i < users.Count; i++)
            {
                for (int j = 0; j < users.Count; j++)
                {
                    //The i==j is forbbiden, and the distance is set to 0
                    if (i != j)
                    {
                        User userI = users[i];
                        User userJ = users[j];

                            distMat[i, j] = EucledianDistance(users[i], users[j]);
                        //Travel time matrices that contain the slope and section for each time period
                        //The travel time is computed using Figliozzi function that actually linearizes
                        // the travel time itslef (but not the travel time bewteen the different time periods - 
                        // that why we linearize it)
                        timeMat[i, j] = distMat[i, j];
                        }
                }
            }
        }

        /*
         * This function returns the lower and upper dicretizied time values (as a double tuple) for specific time period k 
         */
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
                    //Capacity violation - does not depend on time bucket k
                    //tex:$q_i+q_j > C \wedge i \in V_0 \cup F', j \in V_{N+1} \cup F'$
                    if (userI.demand + userJ.demand - loadCap > doublePrecision)
                    {
                        infeasibleArcs[i, j] = true;
                        continue;
                    }
                    //time window violation - does not depend on the time bucket k
                    //tex: $e_i+s_i+t_{ij}>l_j\wedge i \in V_0 \cup F', j \in V_{N+1} \cup F'$
                    if (userI.etw + userI.serviceTime + getTime(userI, userJ) - userJ.ltw > doublePrecision)
                    {
                        infeasibleArcs[i, j] = true;
                        continue;
                    }

                    ////Depot late time windows -does not depend on the time bucket k
                    //tex:  $max(t_i+s_i+t_{ij},e_j)+s_j+t_{jN+1}>l_0 \wedge i \in V_0 \cup F', j \in V \cup F'$
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
                }
            }
        }
    }
}
