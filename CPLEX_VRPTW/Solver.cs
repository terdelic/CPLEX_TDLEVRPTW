using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using ILOG.Concert;
using ILOG.CPLEX;



namespace CPLEX_TDTSPTW
{
    //This is the callback Class used to terminate the search of CPLEX
    // when theory minimal number is found
    internal class TerminateCallback : Cplex.MIPInfoCallback
    {
        double _lastIncumbent;
        int bestTheoryMin;

        internal TerminateCallback(double lastIncumbent, int bestTheoryMin)
        {
            _lastIncumbent = lastIncumbent;
            this.bestTheoryMin = bestTheoryMin;
        }
        public override void Main()
        {
            //Just to check whether the objective function changed or not
            if (HasIncumbent() &&
                 System.Math.Abs(_lastIncumbent - GetIncumbentObjValue())
                     > 1e-5 * (1.0 + System.Math.Abs(GetIncumbentObjValue())))
            {
                _lastIncumbent = GetIncumbentObjValue();
                //If minimal number of vehicles is found, terminate the optimization
                if (Convert.ToInt32(Math.Round(_lastIncumbent, 0)) == bestTheoryMin)
                {
                    Abort();
                }
            }
        }
    }
    internal class Solver
    {
        //Instance of P (usefull)
        Params p;
        //Vehicle minimization
        private int minVehNumOptCPLEX;
        private Cplex.Status statusVehMin;
        private double minVehExecutionTime;

        private Cplex.Status statuSecObjMin;
        private double minSecObjExecutionTime;
        private double minSecObjOptCPLEX;

        private BasicModel bm;
        private MILPData md;
        private Solution bestFind;
        public string outputLine;
        public Solver(Params p)
        {
            this.p = p;
            //Assume that there is no optimization of vehicle number
            minVehNumOptCPLEX = -1;

            Console.WriteLine("Solving TDEVRPTW with full recharge: " + p.instanceName + ", num customers=" + p.customers.Count + ", knownMinVehNum=" + p.knownVehNumberCPLEX + ", minimization objective: " + p.minimizationType.ToString());

            //Create an instance of MILP graph data: vertices, variables etc.
            md = new MILPData(p);
            //Cerate basic model Ax=b
            BasicModel bm = populateModelMatrices();

            int numVehicles = -1;
            //If we do not know the minimal number for problem (the value is -1) we need to perform the vehicle number minimization
            if (p.knownVehNumberCPLEX == -1)
            {
                //Perform vehicle minimization -> PRIMRAY OBJECTIVE
                performeVehicleMin();
                //Only if the solution is feasible or optimal set the found number of vehicles
                if (this.statusVehMin == Cplex.Status.Feasible || this.statusVehMin == Cplex.Status.Optimal)
                {
                    numVehicles = this.minVehNumOptCPLEX;
                }
            }
            else
            {
                numVehicles = p.knownVehNumberCPLEX;
            }

            if (numVehicles == -1 && p.knownVehNumberCPLEX == -1)
            {
                Console.WriteLine("Can not perform distance minimization without known vehicle number!");
            }
            else
            {
                //Perform minimization of second objective: either distance, travel time etc.
                md = new MILPData(p);
                //Cerate basic model Ax=b
                bm = populateModelMatrices();
                performeSecObjectiveMin(bm, numVehicles);
            }
            //Get line string for solution to write it in txt
            outputLine = getSolutionDetails(bestFind);
        }

        public BasicModel populateModelMatrices()
        {
            //Init basic model vectors
            bm = new BasicModel(md.numVars);

            //Upper and lower bounds

            //XIJK - binary decision variable (0,1) integer
            foreach (KeyValuePair<(UserMILP, UserMILP), int> element in md.Xij)
            {
                int varIndexXijk = element.Value;
                bm.xtb[varIndexXijk] = NumVarType.Int;
                bm.lb[varIndexXijk] = 0;
                bm.ub[varIndexXijk] = 1;
            }

            //Users
            foreach (UserMILP um in md.getV0N())
            {
                //Service start time windows bound
                int varIndexTWStart = um.serviceStartTimeVarInd;
                bm.xtb[varIndexTWStart] = NumVarType.Float;
                bm.lb[varIndexTWStart] = um.u.etw;
                bm.ub[varIndexTWStart] = um == md.startDepot ? um.u.etw : um.u.ltw;

                //Load capacity bound
                int varIndexLoad = um.restLoadVarInd;
                bm.xtb[varIndexLoad] = NumVarType.Float;
                bm.lb[varIndexLoad] = um == md.startDepot ? p.loadCap : 0;
                bm.ub[varIndexLoad] = p.loadCap;
            }

            //Equations Ax=b

            /*Only 1 exit arc from each customer 
             *              * */
            //tex: Equation $\forall i \in V$ $\sum_{j \in V_{N+1}, i \neq j} x_{ij}=1$ 
            foreach (UserMILP i in md.getV())
            {
                double[] row = new double[md.numVars];
                foreach (UserMILP j in md.getVN())
                {
                    if (md.Xij.ContainsKey((i, j)))
                    {
                        row[md.Xij[(i, j)]] = 1;
                    }
                }
                if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                {
                    Misc.errOut("All values in a row should not be zero!");
                }
                bm.A.Add(row);
                bm.b.Add(1);
                bm.eqType.Add("equal");
            }

            /*Only 1 input arc from each customer 
 *              * */
            //tex: Equation $\forall i \in V$ $\sum_{j \in V_{0}, i \neq j} x_{ji}=1$ 
            foreach (UserMILP i in md.getV())
            {
                double[] row = new double[md.numVars];
                foreach (UserMILP j in md.getV0())
                {
                    if (md.Xij.ContainsKey((j, i)))
                    {
                        row[md.Xij[(j, i)]] = 1;
                    }
                }
                if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                {
                    Misc.errOut("All values in a row should not be zero!");
                }
                bm.A.Add(row);
                bm.b.Add(1);
                bm.eqType.Add("equal");
            }


            // Flow conservation-> number of exit arcs has to be equal to the number of entry arcs for all arcs in all time periods
            //tex: $\forall j \in V$ $\sum_{i \in V_{N+1}, i \neq j} x_{ji} + \sum_{i \in V_{0}, i \neq j} x_{ij} \leq 1$ 
            foreach (UserMILP i in md.getV())
            {
                foreach (UserMILP j in md.getV())
                {
                    bool any = false;
                    double[] row = new double[md.numVars];
                    if (md.Xij.ContainsKey((j, i)))
                    {
                        row[md.Xij[(j, i)]] += 1;
                        any = true;
                    }
                    if (md.Xij.ContainsKey((i, j)))
                    {
                        any = true;
                        row[md.Xij[(i, j)]] += 1;
                    }
                    if (any)
                    {
                        if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                        {
                            Misc.errOut("All values in a row should not be zero!");
                        }
                        bm.A.Add(row);
                        bm.b.Add(1);
                        bm.eqType.Add("lowerOrEqual");
                    }
                }

            }

            // Equation for time feasibility for arcs leaving customers and depot
            //tex: $\forall i \in V_0, \forall j \in V_{N+1} ~ i\neq j $ $t_i+x_{ij}(s_i+ t_{ij})- l_0(1-x_{ij}) \leq t_j$ 

            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getVN())
                {
                    double[] row = new double[md.numVars];
                    bool any = false;
                    if (md.Xij.ContainsKey((i, j)))
                    {
                        any = true;
                        row[md.Xij[(i, j)]] = p.getTime(i.u, j.u) + p.depot.ltw + i.u.serviceTime;
                    }
                    if (any)
                    {
                        row[i.serviceStartTimeVarInd] = 1;
                        row[j.serviceStartTimeVarInd] = -1;
                        if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                        {
                            Misc.errOut("All values in a row should not be zero!");
                        }
                        bm.A.Add(row);
                        bm.b.Add(p.depot.ltw);
                        bm.eqType.Add("lowerOrEqual");
                    }
                }
            }


            // Equation for reamining load capcity flow conservation
            //tex: $\forall i \in V_0, \forall j \in V_{N+1}~ i\neq j, u_j \leq u_i- x_{ij} (q_i+C)+C$ 

            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getVN())
                {
                    double[] row = new double[md.numVars];
                    row[j.restLoadVarInd] = 1;
                    row[i.restLoadVarInd] = -1;
                    bool any = false;

                    if (md.Xij.ContainsKey((i, j)))
                    {
                        row[md.Xij[(i, j)]] = i.u.demand + p.loadCap;
                        any = true;
                    }
                    if (any)
                    {
                        if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                        {
                            Misc.errOut("All values in a row should not be zero!");
                        }
                        bm.A.Add(row);
                        bm.b.Add(p.loadCap);
                        bm.eqType.Add("lowerOrEqual");
                    }
                }
            }
            return bm;
        }

        /*This function transforms the instance of basic model to the structure used in Cplex itslef
         */
        internal static void PopulateByRow(IMPModeler model,
                                    INumVar[][] var,
                                    IRange[][] rng, Params p, BasicModel bm)
        {
            //Initi model with number of vars, lower bounds, upper bound, and var types
            INumVar[] x = model.NumVarArray(bm.numVars, bm.lb, bm.ub, bm.xtb);
            var[0] = x;
            rng[0] = new IRange[bm.A.Count];
            //Prepare the model to solve equation Ax=b
            for (int i = 0; i < bm.A.Count; i++)
            {
                INumExpr[] expr = new INumExpr[bm.numVars];
                double[] row = bm.A[i];
                //Create product of decision variable x at var index j, with appropirate constant value in row
                for (int j = 0; j < row.Length; j++)
                {
                    expr[j] = model.Prod(row[j], x[j]);
                }
                //Add appropriate ineaqulity/equality type to model
                if (bm.eqType[i] == "equal")
                {
                    rng[0][i] = model.AddEq(model.Sum(expr), bm.b[i]);
                }
                else if (bm.eqType[i] == "lowerOrEqual")
                {
                    rng[0][i] = model.AddLe(model.Sum(expr), bm.b[i]);
                }
                else if (bm.eqType[i] == "greaterOrEqual")
                {
                    rng[0][i] = model.AddGe(model.Sum(expr), bm.b[i]);
                }
                else
                {
                    Misc.errOut("Not impelemented in cplex!");
                }
            }


        }

        public void performeVehicleMin()
        {
            Console.WriteLine("Performing vehicle minimization!");
            int numVeh = 0;
            try
            {
                /*Create an instance of cplex (this to work you have to include
                *ILOG.Concert and ILOG.CPLEX and have several cplex .dll-s in the relative directory
                 */
                Cplex cplex = new Cplex();
                INumVar[][] var = new INumVar[1][];
                IRange[][] rng = new IRange[1][];

                //Transform basic model to appropriate cplex structure
                PopulateByRow(cplex, var, rng, this.p, bm);


                //Create primary object function: vehicle number minimization
                //It can be also written as the number of input arcs in the ending depots, but this is okay
                //tex: $\sum_{k \in K} \sum_{j \in V \cup F'}x_{0j}^k$ 
                int[] objvalsVeh = new int[md.numVars];
                foreach (UserMILP j in md.getV())
                {
                    if (md.Xij.ContainsKey((md.startDepot, j)))
                    {
                        objvalsVeh[md.Xij[(md.startDepot, j)]] = 1;
                    }
                }
                //Minimize
                cplex.AddMinimize(cplex.ScalProd(var[0], objvalsVeh));
                //Set time limit
                cplex.SetParam(Cplex.Param.TimeLimit, p.timeLimitCplex);
                //It was noted that in some cases that the error out of memory occurs (especially on fast computer like toggrupa-01)
                //The default value is 10^75 MB, and here it is limited to a couple of GB (4-8)
                cplex.SetParam(Cplex.Param.MIP.Limits.TreeMemory, p.memoryLimitCplex);
                //In vehicle minimization we call a Terminate callbacke to terminate the optimization when minimal theory number is found
                cplex.Use(new TerminateCallback(double.MaxValue, p.theoryMinNumVehicle));
                //Record starting time (to be able to determine overall execution time)
                double start = cplex.GetCplexTime();
                if (cplex.Solve())
                {
                    //Record execution time
                    minVehExecutionTime = cplex.GetCplexTime() - start;
                    //Get variables
                    double[] x = cplex.GetValues(var[0]);
                    double[] slack = cplex.GetSlacks(rng[0]);
                    cplex.Output().WriteLine("Solution status = " + cplex.GetStatus());
                    cplex.Output().WriteLine("Solution value = " + cplex.ObjValue);

                    //Check the number of vehicles
                    foreach (UserMILP j in md.getV())
                    {
                        if (md.Xij.ContainsKey((md.startDepot, j)))
                        {
                            int index = md.Xij[(md.startDepot, j)];
                            if (x[index] > p.doublePrecision)
                            {
                                numVeh++;
                            }
                        }
                    }
                    if (Convert.ToInt32(cplex.ObjValue) != numVeh)
                    {
                        Misc.errOut("Vehicle number extracted does not comply with cplex vehicle number!");
                    }
                }
                //Record status and the vehicle number
                this.statusVehMin = cplex.GetStatus();
                this.minVehNumOptCPLEX = numVeh;
                cplex.End();
            }
            catch (ILOG.Concert.Exception e)
            {
                Misc.errOut("Concert exception '" + e.Message + "' caught");
            }
        }

        public void performeSecObjectiveMin(BasicModel bm, int numVehicles)
        {
            Console.WriteLine("Performing secondary objective minimization!");

            //Add rows in basic model that limit the number of used vehicles (as we known the feasibl/optimal one from previus minimization)
            // for leaving arcs from a depot
            //tex: $\sum_{j \in V}x_{0j}=numVehicles$ 
            double[] row = new double[md.numVars];
            foreach (UserMILP j in md.getV())
            {
                if (md.Xij.ContainsKey((md.startDepot, j)))
                {
                    int index = md.Xij[(md.startDepot, j)];
                    row[index] = 1;
                }
            }
            if (Misc.EqualDoubleValues(row.Sum(), 0, p.doublePrecision))
            {
                Misc.errOut("Sholud not be all zeros!");
            }
            bm.A.Add(row);
            bm.b.Add(numVehicles);
            bm.eqType.Add("equal");

            //Add rows in basic model that limit the number of used vehicles (as we known the feasibl/optimal one from previus minimization)
            // for entry arcs into a ending depot instances - i think this is actually not necesarry but not sure
            //tex: $\sum_{i \in V \cup F'} \sum_{j \in MED}x_{ij}=numVehicles$ 
            double[] row2 = new double[md.numVars];

            foreach (UserMILP i in md.getV())
            {
                if (md.Xij.ContainsKey((i, md.endDepot)))
                {
                    int index = md.Xij[(i, md.endDepot)];
                    row2[index] = 1;
                }
            }
            if (Misc.EqualDoubleValues(row2.Sum(), 0, p.doublePrecision))
            {
                Misc.errOut("Sholud not be all zeros!");
            }
            bm.A.Add(row2);
            bm.b.Add(numVehicles);
            bm.eqType.Add("equal");

            try
            {
                //Create instance of cplex
                Cplex cplex = new Cplex();
                INumVar[][] var = new INumVar[1][];
                IRange[][] rng = new IRange[1][];
                //Again transfrom basic model (with added equation for vehicle number) to cplex structure
                PopulateByRow(cplex, var, rng, this.p, bm);

                //Create primary object function: vehicle number minimization
                //Distance
                //tex: $\sum_{i \in V_0}  \sum_{j \in V_{N+1}}x_{ij}d_{ij}$


                double[] objvals = new double[md.numVars];
                foreach (UserMILP i in md.getV0())
                {
                    foreach (UserMILP j in md.getVN())
                    {
                        if (md.Xij.ContainsKey((i, j)))
                        {
                            if (p.minimizationType == MinimizationType.Distance)
                            {
                                objvals[md.Xij[(i, j)]] = p.dist(i.u, j.u);
                            }
                        }
                    }
                }
                //Minimize secondary objective
                cplex.AddMinimize(cplex.ScalProd(var[0], objvals));
                //Execution time limit
                cplex.SetParam(Cplex.Param.TimeLimit, p.timeLimitCplex);
                //When it was not working i tried to limit the resolution of a solver to 0.01, but did not help (it was different error=
                //I left it here as a remainder that it can be used
                //cplex.SetParam(Cplex.Param.MIP.Tolerances.Linearization,0.01);
                cplex.SetParam(Cplex.Param.MIP.Limits.TreeMemory, p.memoryLimitCplex);
                double start = cplex.GetCplexTime();
                if (cplex.Solve())
                {
                    //Secondary objective execution time
                    minSecObjExecutionTime = cplex.GetCplexTime() - start;
                    double[] x = cplex.GetValues(var[0]);
                    double[] slack = cplex.GetSlacks(rng[0]);
                    cplex.Output().WriteLine("Secondary objective solution status = " + cplex.GetStatus());
                    cplex.Output().WriteLine("Secondary objective solution value = " + cplex.ObjValue);

                    //Check solution values
                    checkCplexSolutionv2(x, this.md, numVehicles);
                    //Transfrom the cplex array X to the instance of solution
                    bestFind = getSolutionFromCplexV2(x);
                    if (bestFind.vehicles.Count != numVehicles)
                    {
                        Misc.errOut("How is it possible that vehicles differ in second objective minimization!");
                    }

                    if (p.minimizationType == MinimizationType.Distance)
                    {
                        if (Math.Abs(cplex.ObjValue - bestFind.solutionDist) > p.doublePrecision)
                        {
                            Misc.errOut("Distance in the cplex solution and recovered solution do not match!");
                        }
                    }
                    else
                    {
                        Misc.errOut("Unknown secondary objective function!");
                    }

                    Console.WriteLine("Num vehicles:" + bestFind.vehicles.Count);
                    Console.WriteLine("Distance traveled:" + bestFind.solutionDist);
                    Console.WriteLine("Travel time:" + bestFind.solutionTravelTime);
                    Console.WriteLine("Total time:" + bestFind.solutionTotalTime);
                }
                else
                {
                    Console.WriteLine("Unable to find feasible solution!");
                }
                this.statuSecObjMin = cplex.GetStatus();
                this.minSecObjOptCPLEX = cplex.ObjValue;
                cplex.End();
            }
            catch (ILOG.Concert.Exception e)
            {
                Misc.errOut("Concert exception '" + e + "' caught");
            }
        }

        public string getSolutionDetails(Solution s)
        {
            string d = ";";
            //Basic information from optimization
            string line = p.orgSolomonName + d + p.orgNumCust + d + p.BKSVehNum + d + p.BKSCost + d + p.knownVehNumberCPLEX + d + p.minimizationType.ToString() + d +
                this.minVehNumOptCPLEX + d + this.statusVehMin + d +
                (this.minVehExecutionTime / 60.0) + d + this.minSecObjOptCPLEX + d +
                this.statuSecObjMin + d + (minSecObjExecutionTime / 60.0);
            //Get solution details
            if (s != null)
            {
                return line + s.getStrSolutionDetails(d) + "\n";
            }
            else
            {
                return line + d + "NOT_FOUND\n";
            }

        }

        /*
         * This function check wheter the solution returned from cplex is really correct
         * (this is actually selfcheck to see if my model is correct)
         */
        public void checkCplexSolutionv2(double[] x, MILPData md, int numVehs)
        {
            int numInputArcsEndDepots = 0;
            foreach (UserMILP um in md.getV0N())
            {
                //Checking the number of input and exit arc for each user
                int numExit = 0;
                int numInto = 0;
                //Get all exit var indices
                List<int> varIndexXijkOut = md.getAllVarIndicesExitArcsXIJ(um);
                //Get all entry var indices
                List<int> varIndexXijkIn = md.getAllVarIndicesEntryArcsXIJ(um);
                //Count the numbers
                foreach (int varIndex in varIndexXijkOut)
                {
                    if (x[varIndex] > p.doublePrecision)
                    {
                        numExit++;
                    }
                }
                foreach (int varIndex in varIndexXijkIn)
                {
                    if (x[varIndex] > p.doublePrecision)
                    {
                        numInto++;
                    }
                }

                //For starting depot the number of exits arcs has to be equal tu the number of vehicles
                //while the number of input arcs has to be zero
                if (um == md.startDepot)
                {
                    if (numInto != 0 || numExit != numVehs)
                    {
                        Misc.errOut("Arcs at leaving (frist) depot do not match!");
                    }
                }
                //For ending depots the number of input arcs has to be 1, and in total it has to be equal to the number of vehicles
                //while the number of exit arcs has to be zero
                else if (um == md.endDepot)
                {
                    if (numExit != 0 || numInto != numVehs)
                    {
                        Misc.errOut("Arcs at arriving (last) depot do not match!");
                    }
                }
                //For customers the number of input and exit arcs has to be 1
                else if (md.getV().Contains(um))
                {
                    if (numExit != 1 || numInto != 1)
                    {
                        Misc.errOut("User " + um.u._userID + " is either multiple time visited or no at all!");
                    }
                }
                //For stations the number of input and exit arcs has to be equal, and either 0 or 1 (so no larger than 1)
                else
                {
                    Misc.errOut("Wrong set!");
                }

                //Checking rest load capacity for each user
                double restLoadCap = x[um.restLoadVarInd];
                if (restLoadCap < -p.doublePrecision)
                {
                    Misc.errOut("User " + um.u._userID + " has rest load capacity in cplex solution lower than zero!");
                }

                //Checking service begin for each user
                double beginTime = x[um.serviceStartTimeVarInd];
                if (beginTime - um.u.etw < -p.doublePrecision || beginTime - um.u.ltw > p.doublePrecision)
                {
                    Misc.errOut("User " + um.u._userID + " in cplex solution does not satisfy time windows!");
                }
            }
        }

        /*Returns instance of solution extracted from cplex solution array x=A^{-1}b
         */
        private Solution getSolutionFromCplexV2(double[] x)
        {
            //Create instance of solution
            Solution s = new Solution(p);
            int numVeh = 0;
            foreach (KeyValuePair<(UserMILP, UserMILP), int> pair in md.Xij)
            {
                if (x[pair.Value] > p.doublePrecision && pair.Key.Item1 == md.startDepot)
                {
                    numVeh++;
                    //New vehicle
                    Vehicle v = new Vehicle(p, s.vehicles.Count, s);
                    //Arc 0->j
                    UserMILP ui = pair.Key.Item1;
                    UserMILP uj = pair.Key.Item2;
                    while (true)
                    {
                        //If ending depot is found, close vehicle and update all the values
                        if (uj == md.endDepot)
                        {
                            v.AddDepotToEnd();
                            v.updateVehicle();
                            s.addVehicle(v);
                            break;
                        }
                        else
                        {
                            v.route.Add(uj.u);
                            v.updateVehicle();
                            //UserMILP ui = uj;
                            foreach (KeyValuePair<(UserMILP, UserMILP), int> pair2 in md.Xij)
                            {
                                if (pair2.Key.Item1 == uj && x[pair2.Value] > p.doublePrecision)
                                {
                                    //Arc i->j is used
                                    ui = uj;
                                    uj = pair2.Key.Item2;
                                    break;
                                }
                            }
                        }
                    }
                }
            }
            return s;
        }
    }
}

