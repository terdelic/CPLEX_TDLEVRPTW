using ILOG.Concert;


namespace CPLEX_TDTSPTW
{
    /*Class only constructed for the purpose of MILP,
     * that contains the user itself, but aditionaly the
     * indices of coresponding decision variables in array X
     */
    class UserMILP
    {
        public UserMILP(User u)
        {
            this.u = u;
        }
        public User u;
        public int restLoadVarInd;
        public int serviceStartTimeVarInd;
    }

    /* Basic model contains equations for solving Ax=b,
     * with also lower and upper bounds on decision variables
     */
    public class BasicModel
    {
        public double[] lb;
        public double[] ub;
        public NumVarType[] xtb;
        public int numVars;

        public List<double[]> A;
        public List<double> b;
        //This list is used only to know if the equation is equal, lower, greater, lower or equal etc.
        public List<string> eqType;
        public BasicModel(int numVars)
        {
            //Upper and lower bounds
            lb = new double[numVars];
            ub = new double[numVars];
            xtb = new NumVarType[numVars];
            //Equations
            A = new List<double[]>();
            b = new List<double>();
            eqType = new List<string>();
            this.numVars = numVars;
        }
    }


    class MILPData
    {
        //Dictionaries for arc variables
        public Dictionary<(UserMILP, UserMILP), int> Xij;

        //Depots
        public UserMILP startDepot;
        public UserMILP endDepot;
        private List<UserMILP> V;
        public int numVars;

        /*Here go functions to get various list combinations that are used within MILP - see Schneider
         * 
         */
        public List<UserMILP> getV()
        {
            return V;
        }
        public List<UserMILP> getV0()
        {
            List<UserMILP> list = new List<UserMILP>(V);
            list.Add(startDepot);
            return list;
        }

        public List<UserMILP> getVN()
        {
            List<UserMILP> list = new List<UserMILP>(V);
            list.Add(endDepot);
            return list;
        }

        public List<UserMILP> getV0N()
        {
            List<UserMILP> list = new List<UserMILP>(V);
            list.Add(startDepot);
            list.Add(endDepot);
            return list;
        }

        /*Function that returns all appropriate X indices for all arcs that exit user ui
         */
        public List<int> getAllVarIndicesExitArcsXIJ(UserMILP ui)
        {
            return this.Xij.Where(x => x.Key.Item1==ui).Select(x => x.Value).ToList();
        }

        /*Function that returns all appropriate X indices for all arcs that exit user ui and enter user uj
        */
        public List<int> getAllVarIndicesExitArcsXIJ(UserMILP ui, UserMILP uj)
        {
            return this.Xij.Where(x => x.Key.Item1 == ui && x.Key.Item2 == uj).Select(x => x.Value).ToList();
        }

        /*Function that returns all appropriate X indices for all arcs that enter user ui
        */
        public List<int> getAllVarIndicesEntryArcsXIJ(UserMILP ui)
        {
            return this.Xij.Where(x => x.Key.Item2 == ui).Select(x => x.Value).ToList();
        }



        /*
         * Class for storing decision variables and vertices used in MILP program
         * */
        public MILPData(Params p)
        {
            //Dictionary for storinf xijk variable index within MILP
            Xij = new Dictionary<(UserMILP, UserMILP), int>();
            //List of customers
            V = new List<UserMILP>();
            //Load nodes from problem instance
            foreach (User u in p.users)
            {
                if (u.isDepot)
                {
                    //We have two instances of depot: 0-strating depot, N (or N+1) i ending depot
                    startDepot = new UserMILP(p.depot);
                    endDepot = new UserMILP(p.depot);
                }
                else
                {
                    //Customers
                    UserMILP cust = new UserMILP(u);
                    V.Add(cust);
                }
            }

            /*Dictionary for decision variables
             * Xijk - binary variable indicating whether the arc i,j,k is used or not
             * Tijk - float variable indicating the departure time forim user i to user j in time priod k - to be clear only in one period k this value can be >0
             * Bij - binary variable for choosing max between begin of service and early time window (has to be ij)
             * */
            int varIndex = 0;
            foreach (UserMILP uI in getV0())
            {
                foreach (UserMILP uJ in getVN())
                {
                        if (!p.infeas(uI.u, uJ.u))
                        {
                            Xij.Add((uI, uJ), varIndex);
                            varIndex++;
                        }
                    }
            }

            /*For all vertices set decision variables
             * restLoadVarInd - rest load capacity at arrival at user i
             * restEnergyVarInd - rest energy level at ARRIVAL(!) at user I
             * serviceStartTimeVarInd - the start time of service at user i
             */
            foreach (UserMILP u in getV0N())
            {
                u.restLoadVarInd = varIndex;
                varIndex++;
                //Not sure if this decision variable can actually be removed??? (as we have departure time -tricky)
                u.serviceStartTimeVarInd = varIndex;
                varIndex++;
            }
            numVars = varIndex;

            /*Additional explanation
             * Basically we will have an array X that contains all of the decision variables.
             * The length of this array will be numVars, and when we will construct an equalites for the problem
             * we for each vertex in graph that can indicate (rest load, rest energy, or service start time) have to know
             * the exact index in the array X, that coresponds to the appropriate decsion variable.
             * The same goes for arcs. 
             */
        }
    }

}
