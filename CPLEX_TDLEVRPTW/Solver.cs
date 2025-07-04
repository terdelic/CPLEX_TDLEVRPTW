using System;
using System.Collections.Generic;
using System.Diagnostics.Metrics;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using System.Threading.Tasks;
using CPLEX_TDLEVRPTW;
using CPLEX_TDLEVRPTW.CPLEX_TDTSPTW;
using ILOG.Concert;
using ILOG.CPLEX;
using static ILOG.CPLEX.Cplex.Param;



namespace CPLEX_TDTSPTW
{
    //This is the callback Class used to terminate the search of CPLEX
    // when theory minimal number accoridng to CVRP is found
    internal class TerminateCallback : Cplex.MIPInfoCallback
    {
        double lastSolutionValue;
        int bestTheoryMinVehNum;

        internal TerminateCallback(double lastIncumbent, int bestTheoryMin)
        {
            lastSolutionValue = lastIncumbent;
            this.bestTheoryMinVehNum = bestTheoryMin;
        }
        public override void Main()
        {
            //Just to check whether the objective function changed or not
            if (HasIncumbent() &&
                 System.Math.Abs(lastSolutionValue - GetIncumbentObjValue())
                     > 1e-5 * (1.0 + System.Math.Abs(GetIncumbentObjValue())))
            {
                lastSolutionValue = GetIncumbentObjValue();
                //If minimal number of vehicles is found, terminate the optimization
                if (Convert.ToInt32(Math.Round(lastSolutionValue, 0)) == bestTheoryMinVehNum)
                {
                    Abort();
                }
            }
        }
    }
    public partial class Solver
    {
        int helpVar = 0;
        //Instance of Params
        Params p;
        //Vehicle minimization variables
        private int minVehNumOptCPLEX;
        private Cplex.Status statusVehMin;
        private double minVehExecutionTime;
        private Solution bestFindPrim;
        //Secondary objective minimization avriables
        private Cplex.Status statuSecObjMin;
        private double minSecObjExecutionTime;
        private double minSecObjOptCPLEX;
        private Solution bestFindSec;
        //Instances of classes that store the graph values, decision variables and constraints
        //private BasicModel bm;
        //private MILPData md;
        //Additional variables
        public string outputLine;
        private string descriptionLine;

        public int numVars;
        public int numConstraints;
        //Constructor
        public Solver(Params p)
        {
            this.p = p;
            //If value is -1 we assume that there is no optimization of vehicle number
            minVehNumOptCPLEX = -1;
            descriptionLine = "Solving TDEVRPTW with full recharge: " + p.instanceName + ", num customers=" + p.customers.Count +
                ", num virtual CSs=" + p.numMultiCSVert + ", knownMinVehNum=" + p.knownVehNumberCPLEX + ", minimization objective: " + p.minimizationType.ToString() +
                ", Fig Coeffs:" + p.travelTimeCompType;
            Console.WriteLine(descriptionLine);

            //Create an instance of MILP graph data: vertices, variables etc.
            int numEndingDepots = 5; //value of 5 is take as maximum number of vehicles reported by Schneider 2014
            //Create an instance of MILP data
            BasicModel bm;
            MILPData md;
            if (p.modelWithoutCScopies == false)
            {
                md = new MILPDataOld(p, numEndingDepots);
                bm = PopulateBasicModel.populateModelMatricesOld((MILPDataOld)md, p);
            }
            else
            {
                md = new MILPDataNew(p, numEndingDepots);
                bm = PopulateBasicModel.populateModelMatricesNew((MILPDataNew)md, p);

            }
            numVars = md.numVars;
            numConstraints = bm.A.Count;
            //return;
            //Cerate basic model Ax=b


            int numVehicles = -1;
            //If we do not know the minimal number for problem (the value is -1) we need to perform the vehicle number minimization
            if (p.knownVehNumberCPLEX == -1)
            {
                //Perform vehicle minimization -> PRIMRAY OBJECTIVE
                if (p.modelWithoutCScopies == false)
                {
                    performeVehicleMin((MILPDataOld)md, bm);
                }
                else
                {
                    performeVehicleMin((MILPDataNew)md, bm);
                }


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
                Console.WriteLine("Can not perform secondary minimization without known vehicle number!");
            }
            else
            {
                //Perform minimization of secondary objective: either distance, travel time etc.

                if (p.modelWithoutCScopies == false)
                {
                    md = new MILPDataOld(p, numVehicles);
                    bm = PopulateBasicModel.populateModelMatricesOld((MILPDataOld)md, p);
                    performeSecObjectiveMinOld((MILPDataOld)md, bm, numVehicles);
                }
                else
                {
                    md = new MILPDataNew(p, numVehicles);
                    bm = PopulateBasicModel.populateModelMatricesNew((MILPDataNew)md, p);
                    performeSecObjectiveMinNew((MILPDataNew)md, bm, numVehicles);
                }
            }
            Solution bestFind = bestFindSec;
            if (bestFind == null)
            {
                bestFind = bestFindPrim;
            }
            //Get line string for solution to write it in txt
            outputLine = getSolutionDetails(bestFind);
        }



        /*This function transforms the instance of basic model to the structure used in Cplex itslef
         */
        internal static void PopulateByRow(IMPModeler model,
                                    INumVar[][] var,
                                    IRange[][] rng, Params p, BasicModel bm, MILPDataNew md = null)
        {

            //List<UserMILP> usedEndingDepots=new List<UserMILP> ();
            //string ssol = "0|5|2|6|0--0|3|8|4|1|7|0";
            //Solution s = new Solution(p);
            //string[] split = ssol.Split("--");
            //for(int i = 0; i < split.Length; i++)
            //{
            //    Vehicle v = new Vehicle(p, i, s);
            //    string[] spli2 = split[i].Split('|');
            //    for(int j=1;j< spli2.Length; j++)
            //    {
            //        int userID = Convert.ToInt32(spli2[j]);
            //        User u = p.users[userID];
            //        if (u.isDepot)
            //        {
            //            v.AddDepotToEnd();
            //        }
            //        else
            //        {
            //            v.route.Add(u);
            //        }

            //    }
            //    v.updateVehicle();
            //    s.addVehicle(v);
            //}
            //double[] xCheck = new double[md.numVars];
            //foreach (Vehicle v in s.vehicles)
            //{
            //    User ui = v.route[0];
            //    for (int i = 1; i < v.route.Count; i++)
            //    {
            //        User uj = v.route[i];
            //        User CS = null;
            //        if (v.route[i].isStation())
            //        {
            //            uj = v.route[i + 1];
            //            CS = v.route[i];
            //        }

            //        double departTimeI = ui.departureTime;
            //        int kXIJK = p.getTimeIntervalIndex(departTimeI);
            //        UserMILP uim = md.getV0().Where(u => u.u._userID == ui._userID).FirstOrDefault();
            //        UserMILP ujm = md.getV_ED().Where(u => u.u._userID == uj._userID).FirstOrDefault();
            //        if (uj.isDepot)
            //        {
            //            List<UserMILP> listujm = md.getMED();
            //            foreach (UserMILP uend in listujm)
            //            {
            //                if (!usedEndingDepots.Contains(uend))
            //                {
            //                    usedEndingDepots.Add(uend);
            //                    ujm = uend;
            //                    break;
            //                }
            //            }
            //        }
            //        xCheck[md.Xijk[(uim, ujm, kXIJK)]] = 1;
            //        xCheck[md.Zetaijk[(uim, ujm, kXIJK)]] = ui.departureTime;
            //        xCheck[uim.serviceStartTimeVarInd] = ui.beginTime;
            //        xCheck[ujm.serviceStartTimeVarInd] = uj.beginTime;
            //        xCheck[uim.restLoadVarInd] = ui.restLoadCap+ui.demand;
            //        xCheck[ujm.restLoadVarInd] = uj.restLoadCap+uj.demand;
            //        xCheck[uim.restEnergyVarInd] = ui.restBatCapAtArrival;
            //        xCheck[ujm.restEnergyVarInd] = uj.restBatCapAtArrival;
            //        if (ujm.u.arrivalTime < ujm.u.etw)
            //        {
            //            xCheck[md.Bij[(uim, ujm)]] = 1;
            //        }
            //        else
            //        {
            //            xCheck[md.Bij[(uim, ujm)]] = 0;
            //        }

            //        if (CS != null)
            //        {
            //            int kZISJK =kXIJK;
            //            UserMILP usm = md.getF().Where(u => u.u._userID == CS._userID).FirstOrDefault();
            //            xCheck[md.Zisjk[(uim,usm, ujm, kZISJK)]] = 1;

            //            double departTimeCS = CS.departureTime;
            //            int kZ_ISJK = p.getTimeIntervalIndex(departTimeCS);
            //            xCheck[md.Z_isjk[(uim, usm, ujm, kZ_ISJK)]] = 1;
            //            xCheck[md.Zetaisjk[(uim, usm, ujm, kZ_ISJK)]] = CS.departureTime;
            //            xCheck[md.Yisj[(uim, usm, ujm)]] = CS.restBatCapAtArrival;
            //            xCheck[md.Deltaisj[(uim, usm, ujm)]] = departTimeCS-departTimeI+  p.LinTime(usm.u, ujm.u, departTimeCS)- p.LinTime(uim.u, ujm.u, departTimeI);


            //        }
            //        else
            //        {
            //            ui = uj;
            //        }

            //    }
            //}


            //Initi model with number of vars, lower bounds, upper bound, and var types
            INumVar[] x = model.NumVarArray(bm.numVars, bm.lb, bm.ub, bm.xtb);
            var[0] = x;
            rng[0] = new IRange[bm.A.Count];
            //Prepare the model to solve equation Ax=b
            for (int i = 0; i < bm.A.Count; i++)
            {
                INumExpr[] expr = new INumExpr[bm.numVars];
                double[] row = bm.A[i];
                //double sumCheck = 0;
                //Create product of decision variable x at var index j, with appropirate constant value in row
                for (int j = 0; j < row.Length; j++)
                {
                    expr[j] = model.Prod(row[j], x[j]);
                    //sumCheck += row[j] * xCheck[j];
                }
                //Add appropriate ineaqulity/equality type to model
                if (bm.eqType[i] == "equal")
                {
                    rng[0][i] = model.AddEq(model.Sum(expr), bm.b[i]);
                    //if(!Misc.EqualDoubleValues(sumCheck, bm.b[i], p.doublePrecision))
                    //{
                    //    Misc.errOut($"Constraint equal {i} is not satisfied");
                    //}
                }
                else if (bm.eqType[i] == "lowerOrEqual")
                {
                    rng[0][i] = model.AddLe(model.Sum(expr), bm.b[i]);
                    //if (sumCheck - bm.b[i]>p.doublePrecision)
                    //{
                    //     Misc.errOut($"Constraint lower or equal {i} is not satisfied");
                    //}
                }
                else if (bm.eqType[i] == "greaterOrEqual")
                {
                    rng[0][i] = model.AddGe(model.Sum(expr), bm.b[i]);
                    //if (sumCheck - bm.b[i] < -p.doublePrecision)
                    //{
                    //    Misc.errOut($"Constraint greater or equal {i} is not satisfied");
                    //}
                }
                else
                {
                    Misc.errOut("Not impelemented in cplex!");
                }
            }
        }

        //Method to perform primary minimization - vehicle number
        public void performeVehicleMin(MILPData mdin, BasicModel bm)
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
                PopulateByRow(cplex, var, rng, this.p, bm); //cplex, var, rng, this.p, bm, (MILPDataNew)mdin


                //Create primary object function: vehicle number minimization
                //It can be also written as the number of input arcs in the ending depots, but this is okay
                int[] objvalsVeh;
                if (mdin.newOrOld == "Old")
                {
                    MILPDataOld md = (MILPDataOld)mdin;
                    objvalsVeh = new int[md.numVars];
                    //tex: $\sum_{k \in K} \sum_{j \in V \cup F'}x_{0j}^k$ 

                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        foreach (UserMILP j in md.getV_())
                        {
                            if (md.Xijk.ContainsKey((md.depot0, j, k)))
                            {
                                objvalsVeh[md.Xijk[(md.depot0, j, k)]] = 1;
                            }
                        }
                    }
                }
                else
                {

                    MILPDataNew md = (MILPDataNew)mdin;
                    objvalsVeh = new int[md.numVars];
                    //tex: $\sum_{k \in K} \sum_{j \in V}x_{0j}^k$ 

                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        foreach (UserMILP j in md.getV())
                        {
                            if (md.Xijk.ContainsKey((md.depot0, j, k)))
                            {
                                objvalsVeh[md.Xijk[(md.depot0, j, k)]] = 1;
                            }
                        }
                    }
                }

                //Minimize
                cplex.AddMinimize(cplex.ScalProd(var[0], objvalsVeh));
                //Set time limit
                cplex.SetParam(Cplex.Param.TimeLimit, p.timeLimitCplex);
                //It was noted that in some cases that the error out of memory occurs (especially on fast computer like toggrupa-01)
                //The default value is large, and here it is limited to a couple of GB (4-8)
                cplex.SetParam(Cplex.Param.MIP.Limits.TreeMemory, p.memoryLimitCplex);
                //In vehicle minimization we call a Terminate callbacke to terminate the optimization when minimal theory number is found
                cplex.Use(new TerminateCallback(double.MaxValue, p.theoryMinNumVehicle));
                //Record starting time (to be able to determine overall execution time)
                double start = cplex.GetCplexTime();
                if (cplex.Solve())
                {
                    //Record execution time
                    minVehExecutionTime = cplex.GetCplexTime() - start;
                    //Get status
                    this.statusVehMin = cplex.GetStatus();
                    cplex.Output().WriteLine("Solution status = " + this.statusVehMin);
                    //Store the oslution only if its feasible and optimal
                    if (this.statusVehMin == Cplex.Status.Feasible || this.statusVehMin == Cplex.Status.Optimal)
                    {
                        double[] x = cplex.GetValues(var[0]);
                        //double[] slack = cplex.GetSlacks(rng[0]);
                        cplex.Output().WriteLine("Solution value = " + cplex.ObjValue);
                        //Check the number of vehicles
                        if (mdin.newOrOld == "Old")
                        {
                            MILPDataOld md = (MILPDataOld)mdin;
                            for (int k = 0; k < md.kBuckets; k++)
                            {
                                foreach (UserMILP j in md.getV_())
                                {
                                    if (md.Xijk.ContainsKey((md.depot0, j, k)))
                                    {
                                        int index = md.Xijk[(md.depot0, j, k)];
                                        if (x[index] > p.doublePrecision)
                                        {
                                            numVeh++;
                                        }
                                    }
                                }
                            }
                        }
                        else
                        {
                            MILPDataNew md = (MILPDataNew)mdin;
                            for (int k = 0; k < md.kBuckets; k++)
                            {
                                foreach (UserMILP j in md.getV())
                                {
                                    if (md.Xijk.ContainsKey((md.depot0, j, k)))
                                    {
                                        int index = md.Xijk[(md.depot0, j, k)];
                                        if (x[index] > p.doublePrecision)
                                        {
                                            numVeh++;
                                        }
                                    }
                                }
                            }
                        }
                        if (Convert.ToInt32(cplex.ObjValue) != numVeh)
                        {
                            Misc.errOut("Vehicle number extracted does not match with cplex vehicle number!");
                        }
                        //Check the solution according to CPLEX variables
                        if (mdin.newOrOld == "Old")
                        {
                            MILPDataOld md = (MILPDataOld)mdin;
                            checkCplexSolutionOld(x, md, numVeh, false);
                            //Transfrom the cplex array X to get the instance of solution
                            bestFindPrim = getSolutionFromCplexOld(md, x);
                        }
                        else
                        {
                            MILPDataNew md = (MILPDataNew)mdin;
                            checkCplexSolutionNew(x, md, numVeh, false);
                            //Transfrom the cplex array X to get the instance of solution
                            bestFindPrim = getSolutionFromCplexNew(md, x);
                        }

                        //Record vehicle number
                        this.minVehNumOptCPLEX = numVeh;
                    }
                }
                //End cplex instance
                cplex.End();
            }
            catch (ILOG.Concert.Exception e)
            {
                Misc.errOut("Error when minimizing number of vehicles! Concert exception '" + e.Message + "' caught");
            }
        }

        //Method for sescondary objective
        public void performeSecObjectiveMinOld(MILPDataOld md, BasicModel bm, int numVehicles)
        {
            Console.WriteLine("Performing secondary objective minimization!");

            //Add rows in basic model that limit the number of used vehicles (as we known the feasible/optimal or set ones from previus minimization/confgi file)
            // for leaving arcs from a depot
            //tex: $\sum_{k \in K} \sum_{j \in V \cup F'}x_{0j}^k=numVehicles$ 
            double[] row = new double[md.numVars];
            for (int k = 0; k < md.kBuckets; k++)
            {
                foreach (UserMILP j in md.getV_())
                {
                    if (md.Xijk.ContainsKey((md.depot0, j, k)))
                    {
                        int index = md.Xijk[(md.depot0, j, k)];
                        row[index] = 1;
                    }
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
            // for entry arcs into a ending depot instances - I think this is actually not necesarry but not sure
            //tex: $\sum_{k \in K} \sum_{i \in V \cup F'} \sum_{j \in MED}x_{ij}^k=numVehicles$ 
            double[] row2 = new double[md.numVars];
            for (int k = 0; k < md.kBuckets; k++)
            {
                foreach (UserMILP i in md.getV_())
                {
                    foreach (UserMILP j in md.getMED())
                    {
                        if (md.Xijk.ContainsKey((i, j, k)))
                        {
                            int index = md.Xijk[(i, j, k)];
                            row2[index] = 1;
                        }
                    }
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
                //tex: $\sum_{k \in K}\sum_{i \in V_0 \cup F'}  \sum_{j \in V_{ED} \cup F'}x_{ij}^kd_{ij}^k$

                //Travel time
                //tex: $\sum_{k \in K}\sum_{i \in V_0 \cup F'}  \sum_{j \in V_{ED} \cup F'}\zeta_{ij}^k \Theta_{ij}^k +x_{ij}^k \mu_{ij}^k$

                //Total time - this equation should be okay in all cases
                //tex: $\sum_{k \in K}\sum_{i \in MED} \tau_{i}^k$

                double[] objvals = new double[md.numVars];
                for (int k = 0; k < md.kBuckets; k++)
                {
                    foreach (UserMILP i in md.getV0_())
                    {
                        foreach (UserMILP j in md.getVNm_())
                        {
                            if (md.Xijk.ContainsKey((i, j, k)))
                            {
                                if (p.minimizationType == MinimizationType.Distance)
                                {
                                    objvals[md.Xijk[(i, j, k)]] = p.dist(i.u, j.u, k);
                                }
                                else if (p.minimizationType == MinimizationType.TravelTime)
                                {
                                    objvals[md.Xijk[(i, j, k)]] = p.getSectionFromLinTime(i.u, j.u, k);
                                    objvals[md.Tijk[(i, j, k)]] = p.getSlopeFromLinTime(i.u, j.u, k);
                                }
                            }
                        }
                    }
                    if (p.minimizationType == MinimizationType.TotalTime)
                    {
                        foreach (UserMILP i in md.endingDepots)
                        {
                            objvals[i.serviceStartTimeVarInd] = 1;
                        }
                    }
                }
                //Minimize secondary objective
                cplex.AddMinimize(cplex.ScalProd(var[0], objvals));
                //Execution time limit
                cplex.SetParam(Cplex.Param.TimeLimit, p.timeLimitCplex);
                //When it was not working i tried to limit the resolution of a solver to 0.01, but did not help (it was different error)
                //I left it here as a remainder that it can be used
                //cplex.SetParam(Cplex.Param.MIP.Tolerances.Linearization,0.00000001);
                cplex.SetParam(Cplex.Param.MIP.Limits.TreeMemory, p.memoryLimitCplex);
                double start = cplex.GetCplexTime();
                if (cplex.Solve())
                {
                    //Secondary objective execution time
                    minSecObjExecutionTime = cplex.GetCplexTime() - start;
                    this.statuSecObjMin = cplex.GetStatus();
                    cplex.Output().WriteLine("Secondary objective solution status = " + cplex.GetStatus());
                    //Store variabels only in solution is feasible or optimal
                    if (this.statuSecObjMin == Cplex.Status.Feasible || this.statuSecObjMin == Cplex.Status.Optimal)
                    {
                        this.minSecObjOptCPLEX = cplex.ObjValue;
                        double[] x = cplex.GetValues(var[0]);
                        double[] slack = cplex.GetSlacks(rng[0]);
                        cplex.Output().WriteLine("Secondary objective solution value = " + cplex.ObjValue);
                        //Check solution values
                        checkCplexSolutionOld(x, md, numVehicles, true);
                        //Transfrom the cplex array X to the instance of solution
                        bestFindSec = getSolutionFromCplexOld(md, x);
                        //Checkers
                        if (bestFindSec.vehicles.Count != numVehicles)
                        {
                            Misc.errOut("How is it possible that vehicles differ in second objective minimization!");
                        }
                        if (p.minimizationType == MinimizationType.Distance)
                        {
                            bestFindSec.diffBetweenCplexAndReal = cplex.ObjValue - bestFindSec.solutionDist;
                            if (Math.Abs(cplex.ObjValue - bestFindSec.solutionDist) > p.doublePrecision)
                            {
                                Misc.errOut("Distance in the cplex solution and recovered solution do not match!");
                            }
                        }
                        else if (p.minimizationType == MinimizationType.TravelTime)
                        {
                            bestFindSec.diffBetweenCplexAndReal = cplex.ObjValue - bestFindSec.solutionTravelTime;
                            if (Math.Abs(cplex.ObjValue - bestFindSec.solutionTravelTime) > p.doublePrecision)
                            {
                                Misc.errOut("Travel time in the cplex solution and recovered solution do not match!");
                            }
                        }
                        else if (p.minimizationType == MinimizationType.TotalTime)
                        {
                            bestFindSec.diffBetweenCplexAndReal = cplex.ObjValue - bestFindSec.solutionTotalTime;
                            if (Math.Abs(cplex.ObjValue - bestFindSec.solutionTotalTime) > p.doublePrecision)
                            {
                                Misc.errOut("Total time in the cplex solution and recovered solution do not match!");
                            }
                        }
                        else
                        {
                            Misc.errOut("Unknown secondary objective function!");
                        }
                        //Write solution valuse into the console
                        Console.WriteLine("Num vehicles:" + bestFindSec.vehicles.Count);
                        Console.WriteLine("Distance traveled:" + bestFindSec.solutionDist);
                        Console.WriteLine("Travel time:" + bestFindSec.solutionTravelTime);
                        Console.WriteLine("Energy consumed:" + bestFindSec.solutionDist);
                        Console.WriteLine("Total time:" + bestFindSec.solutionTotalTime);
                    }
                    else
                    {
                        Console.WriteLine("Unable to find feasible solution!");
                    }
                }
                cplex.End();
            }
            catch (ILOG.Concert.Exception e)
            {
                Misc.errOut("Error when minimizing secondary objective! Concert exception '" + e + "' caught");
            }
        }


        public void performeSecObjectiveMinNew(MILPDataNew md, BasicModel bm, int numVehicles)
        {
            Console.WriteLine("Performing secondary objective minimization!");

            //Add rows in basic model that limit the number of used vehicles (as we known the feasible/optimal or set ones from previus minimization/confgi file)
            // for leaving arcs from a depot
            //tex: $\sum_{k \in K} \sum_{j \in V}x_{0j}^k=numVehicles$ 
            double[] row = new double[md.numVars];
            for (int k = 0; k < md.kBuckets; k++)
            {
                foreach (UserMILP j in md.getV())
                {
                    if (md.Xijk.ContainsKey((md.depot0, j, k)))
                    {
                        int index = md.Xijk[(md.depot0, j, k)];
                        row[index] = 1;
                    }
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
            // for entry arcs into a ending depot instances - I think this is actually not necesarry but not sure
            //tex: $\sum_{k \in K} \sum_{i \in V} \sum_{j \in MED}x_{ij}^k=numVehicles$ 
            double[] row2 = new double[md.numVars];
            for (int k = 0; k < md.kBuckets; k++)
            {
                foreach (UserMILP i in md.getV())
                {
                    foreach (UserMILP j in md.getMED())
                    {
                        if (md.Xijk.ContainsKey((i, j, k)))
                        {
                            int index = md.Xijk[(i, j, k)];
                            row2[index] = 1;
                        }
                    }
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
                //tex: $\sum_{k \in K}\sum_{i \in V_0}  \sum_{j \in V_{ED}}x_{ij}^kd_{ij}^k + \sum_{k \in K} \sum_{i \in V_0}  \sum_{j \in V_{ED}} \sum_{s \in F} (z_{isj}^k d_{is}^k+ \overline{z}_{isj}^k d_{sj}^k-z_{isj}^k d_{ij}^k)$

                //Travel time
                //tex: $\sum_{k \in K}\sum_{i \in V_0}  \sum_{j \in V_{ED}} (\zeta_{ij}^k \Theta_{ij}^k +x_{ij}^k \eta_{ij}^k) + \sum_{i \in V_0}  \sum_{j \in V_{ED}} \sum_{s \in F} \Delta_{isj}$


                double[] objvals = new double[md.numVars];

                foreach (UserMILP i in md.getV0())
                {
                    foreach (UserMILP j in md.getV_ED())
                    {
                        for (int k = 0; k < md.kBuckets; k++)
                        {
                            if (md.Xijk.ContainsKey((i, j, k)))
                            {
                                if (p.minimizationType == MinimizationType.Distance)
                                {
                                    objvals[md.Xijk[(i, j, k)]] = p.dist(i.u, j.u, k);
                                    foreach (UserMILP s in md.getF())
                                    {
                                        if (md.Zisjk.ContainsKey((i, s, j, k)) && md.Z_isjk.ContainsKey((i, s, j, k)))
                                        {
                                            objvals[md.Zisjk[(i, s, j, k)]] = p.dist(i.u, s.u, k) - p.dist(i.u, j.u, k);
                                            objvals[md.Z_isjk[(i, s, j, k)]] = p.dist(s.u, j.u, k);
                                        }
                                    }
                                }
                                else if (p.minimizationType == MinimizationType.TravelTime)
                                {
                                    objvals[md.Xijk[(i, j, k)]] = p.getSectionFromLinTime(i.u, j.u, k);
                                    objvals[md.Zetaijk[(i, j, k)]] = p.getSlopeFromLinTime(i.u, j.u, k);


                                }

                                //else if (p.minimizationType == MinimizationType.TravelTime)
                                //{
                                //    objvals[md.Xijk[(i, j, k)]] = p.getSectionFromLinTime(i.u, j.u, k);
                                //    objvals[md.Zetaijk[(i, j, k)]] = p.getSlopeFromLinTime(i.u, j.u, k);

                                //    foreach (UserMILP s in md.getF())
                                //    {
                                //        if (md.Zisjk.ContainsKey((i, s, j, k)) && md.Z_isjk.ContainsKey((i, s, j, k)) &&
                                //            md.Zetaisjk.ContainsKey((i, s, j, k)))
                                //        {
                                //            objvals[md.Zetaijk[(i, j, k)]] = p.getSlopeFromLinTime(i.u, s.u, k);
                                //            objvals[md.Zisjk[(i, s, j, k)]] = p.getSectionFromLinTime(i.u, s.u, k);
                                //            objvals[md.Zetaisjk[(i, s, j, k)]] = p.getSlopeFromLinTime(s.u, j.u, k);
                                //            objvals[md.Z_isjk[(i, s, j, k)]] = p.getSectionFromLinTime(s.u, j.u, k);
                                //        }
                                //    }
                                //}
                            }
                        }
                        if (p.minimizationType == MinimizationType.TravelTime)
                        {
                            foreach (UserMILP s in md.getF())
                            {
                                if (md.Deltaisj.ContainsKey((i, s, j)) && md.Yisj.ContainsKey((i, s, j)))
                                {
                                    objvals[md.Deltaisj[(i, s, j)]] = 1;
                                    objvals[md.Yisj[(i, s, j)]] = p.refuelRate;

                                    for (int k = 0; k < md.kBuckets; k++)
                                    {
                                        if (md.Zisjk.ContainsKey((i, s, j,k)))
                                        {
                                            objvals[md.Zisjk[(i, s, j,k)]] = -p.refuelRate*p.batCap;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    //if (p.minimizationType == MinimizationType.TotalTime)
                    //{
                    //    foreach (UserMILP i in md.endingDepots)
                    //    {
                    //        objvals[i.serviceStartTimeVarInd] = 1;
                    //    }
                    //}
                }
                //Minimize secondary objective
                cplex.AddMinimize(cplex.ScalProd(var[0], objvals));
                //Execution time limit
                cplex.SetParam(Cplex.Param.TimeLimit, p.timeLimitCplex);
                //When it was not working i tried to limit the resolution of a solver to 0.01, but did not help (it was different error)
                //I left it here as a remainder that it can be used
                //cplex.SetParam(Cplex.Param.MIP.Tolerances.Linearization,0.00000001);
                cplex.SetParam(Cplex.Param.MIP.Limits.TreeMemory, p.memoryLimitCplex);
                double start = cplex.GetCplexTime();
                if (cplex.Solve())
                {
                    //Secondary objective execution time
                    minSecObjExecutionTime = cplex.GetCplexTime() - start;
                    this.statuSecObjMin = cplex.GetStatus();
                    cplex.Output().WriteLine("Secondary objective solution status = " + cplex.GetStatus());
                    //Store variabels only in solution is feasible or optimal
                    if (this.statuSecObjMin == Cplex.Status.Feasible || this.statuSecObjMin == Cplex.Status.Optimal)
                    {
                        this.minSecObjOptCPLEX = cplex.ObjValue;
                        double[] x = cplex.GetValues(var[0]);
                        double[] slack = cplex.GetSlacks(rng[0]);
                        cplex.Output().WriteLine("Secondary objective solution value = " + cplex.ObjValue);
                        //Check solution values
                        checkCplexSolutionNew(x, md, numVehicles, true);
                        //Transfrom the cplex array X to the instance of solution
                        bestFindSec = getSolutionFromCplexNew(md, x);
                        //Checkers
                        if (bestFindSec.vehicles.Count != numVehicles)
                        {
                            Misc.errOut("How is it possible that vehicles differ in second objective minimization!");
                        }
                        if (p.minimizationType == MinimizationType.Distance)
                        {
                            bestFindSec.diffBetweenCplexAndReal = cplex.ObjValue - bestFindSec.solutionDist;
                            if (Math.Abs(cplex.ObjValue - bestFindSec.solutionDist) > p.doublePrecision)
                            {
                                Misc.errOut("Distance in the cplex solution and recovered solution do not match!");
                            }
                        }
                        else if (p.minimizationType == MinimizationType.TravelTime)
                        {
                            bestFindSec.diffBetweenCplexAndReal = cplex.ObjValue - bestFindSec.solutionTravelTime;
                            if (Math.Abs(cplex.ObjValue - bestFindSec.solutionTravelTime) > p.doublePrecision)
                            {
                                Misc.errOut("Travel time in the cplex solution and recovered solution do not match!");
                            }
                        }
                        else if (p.minimizationType == MinimizationType.TotalTime)
                        {
                            bestFindSec.diffBetweenCplexAndReal = cplex.ObjValue - bestFindSec.solutionTotalTime;
                            if (Math.Abs(cplex.ObjValue - bestFindSec.solutionTotalTime) > p.doublePrecision)
                            {
                                Misc.errOut("Total time in the cplex solution and recovered solution do not match!");
                            }
                        }
                        else
                        {
                            Misc.errOut("Unknown secondary objective function!");
                        }
                        //Write solution valuse into the console
                        Console.WriteLine("Num vehicles:" + bestFindSec.vehicles.Count);
                        Console.WriteLine("Distance traveled:" + bestFindSec.solutionDist);
                        Console.WriteLine("Travel time:" + bestFindSec.solutionTravelTime);
                        Console.WriteLine("Energy consumed:" + bestFindSec.solutionDist);
                        Console.WriteLine("Total time:" + bestFindSec.solutionTotalTime);
                    }
                    else
                    {
                        Console.WriteLine("Unable to find feasible solution!");
                    }
                }
                cplex.End();
            }
            catch (ILOG.Concert.Exception e)
            {
                Misc.errOut("Error when minimizing secondary objective! Concert exception '" + e + "' caught");
            }
        }

        //Method to get solution details as  string
        public string getSolutionDetails(Solution s)
        {
            string d = ";";
            //Basic information from optimization
            string line = p.orgSolomonName + d + p.orgNumCust + d + p.BKSVehNum + d + p.BKSCost + d + p.numMultiCSVert +
                d + p.knownVehNumberCPLEX + d + p.minimizationType.ToString() + d +
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
         * This function checks wheter the solution returned from cplex is really correct
         * (this is actually selfcheck to see if my model is correct)
         */
        public void checkCplexSolutionOld(double[] x, MILPDataOld md, int numVehs, bool second)
        {
            int numInputArcsEndDepots = 0;
            foreach (UserMILP um in md.getV0Nm_())
            {
                //Checking the number of input and exit arc for each user
                int numExit = 0;
                int numInto = 0;
                //Get all exit var indices
                List<int> varIndexXijkOut = md.getAllVarIndicesExitArcsXIJK(um);
                //Get all entry var indices
                List<int> varIndexXijkIn = md.getAllVarIndicesEntryArcsXIJK(um);
                //Count the numbers
                foreach (int varIndex in varIndexXijkOut)
                {
                    //Has to be larger than p.doublePrecision because some values, althoug marked as integer bettween [0,1] have values for example 10^-5
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
                if (um == md.depot0)
                {
                    if (numInto != 0 || numExit != numVehs)
                    {
                        Misc.errOut("Arcs at leaving (frist) depot do not match!");
                    }
                }
                //For ending depots the number of input arcs has to be 1, and in total it has to be equal to the number of vehicles
                //while the number of exit arcs has to be zero
                else if (md.endingDepots.Contains(um))
                {
                    if (numExit != 0 || (numInto != 1 && second) || (numInto > 1 && !second))
                    {
                        Misc.errOut("Arcs at arriving (last) depot do not match!");
                    }
                    numInputArcsEndDepots += numInto;
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
                else if (md.getF_().Contains(um))
                {
                    if (numExit > 1 || numInto > 1 || numExit != numInto)
                    {
                        Misc.errOut("User " + um.u._userID + " is either multiple time visited or no at all!");
                    }
                }
                else
                {
                    Misc.errOut("Wrong set!");
                }

                //Checking rest battery capacity for each user
                double restBatCap = x[um.restEnergyVarInd];
                if (restBatCap < -p.doublePrecision)
                {
                    Misc.errOut("User " + um.u._userID + " has rest bat capacity in cplex solution lower than zero!");
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
                //Checking departure time for each user excpet ending depot which actually does not have a departure time
                if (!md.endingDepots.Contains(um))
                {
                    int count = 0;
                    double departureTime = 0;
                    foreach (KeyValuePair<(UserMILP, UserMILP, int), int> pair in md.Tijk)
                    {
                        if (x[pair.Value] > p.doublePrecision && pair.Key.Item1 == um)
                        {
                            if (x[md.Xijk[(pair.Key.Item1, pair.Key.Item2, pair.Key.Item3)]] > p.doublePrecision)
                            {
                                count++;
                                departureTime += x[pair.Value];
                            }

                        }
                    }
                    //If departure time is used for a user is used multiple times
                    // this would mean that he starts several times, and this was a big question
                    // did i model this part okay or not, as it has to be used only once per user (in all time periods)                    
                    if (count > 1)
                    {
                        Misc.errOut("Departure time variable is used multiple times!");
                    }
                    //For customer the departure time has to be equal to begintime+serviceTime
                    double computedDepartureTime = beginTime;
                    if (md.getV().Contains(um) || um == md.depot0)
                    {
                        computedDepartureTime += um.u.serviceTime;
                    }
                    //For station the departure time has to be equal to begintime+chargingTime (acquired from the decision variable)
                    else if (md.getF_().Contains(um))
                    {
                        computedDepartureTime += p.refuelRate * (p.batCap - restBatCap);
                    }
                    else
                    {
                        Misc.errOut("Wrong set!");
                    }
                    //Compare departure times
                    if (Math.Abs(departureTime - computedDepartureTime) > p.doublePrecision)
                    {
                        Misc.errOut("Departure times in cplex solution do not match!");
                    }
                }
            }
            //Often when double resolution is lower 10^-7 the xijk value has a zero, or near zero value (10^-8).
            //The problem occurs when the value is 10^-8, then corresponding Tijk value can have a valeu larger than zero, i.e. 10^-5
            //which lead to errors when comparing extracted solution and cplex solution, thats why we decreased the double resoltion to 10^-3
            foreach (KeyValuePair<(UserMILP, UserMILP, int), int> pair in md.Xijk)
            {
                if (x[pair.Value] <= p.doublePrecision)
                {
                    int indextijkVal = md.Tijk[(pair.Key.Item1, pair.Key.Item2, pair.Key.Item3)];
                    if (x[indextijkVal] > p.doublePrecision)
                    {
                        Misc.errOut("The xijk values is zero, while the tijk values is larger than zero!");
                    }
                }
            }
            //Compare number of arcs for ending depots
            if (numInputArcsEndDepots != numVehs)
            {
                Misc.errOut("Number of input arcs into ending depots differs from num of vehicles!");
            }
        }



        public void checkCplexSolutionNew(double[] x, MILPDataNew md, int numVehs, bool second)
        {
            int numInputArcsEndDepots = 0;
            foreach (UserMILP um in md.getV0_cup_ED())
            {
                //Checking the number of input and exit arc for each user
                int numExit = 0;
                int numInto = 0;
                //Get all exit var indices
                List<int> varIndexXijkOut = md.getAllVarIndicesExitArcsXIJK(um);
                //Get all entry var indices
                List<int> varIndexXijkIn = md.getAllVarIndicesEntryArcsXIJK(um);
                //Count the numbers
                foreach (int varIndex in varIndexXijkOut)
                {
                    //Has to be larger than p.doublePrecision because some values, althoug marked as integer bettween [0,1] have values for example 10^-5
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
                if (um == md.depot0)
                {
                    if (numInto != 0 || numExit != numVehs)
                    {
                        Misc.errOut("Arcs at leaving (frist) depot do not match!");
                    }
                }
                //For ending depots the number of input arcs has to be 1, and in total it has to be equal to the number of vehicles
                //while the number of exit arcs has to be zero
                else if (md.endingDepots.Contains(um))
                {
                    if (numExit != 0 || (numInto != 1 && second) || (numInto > 1 && !second))
                    {
                        Misc.errOut("Arcs at arriving (last) depot do not match!");
                    }
                    numInputArcsEndDepots += numInto;
                }
                //For customers the number of input and exit arcs has to be 1
                else if (md.getV().Contains(um))
                {
                    if (numExit != 1 || numInto != 1)
                    {
                        Misc.errOut("User " + um.u._userID + " is either multiple time visited or no at all!");
                    }
                }
                else
                {
                    Misc.errOut("Wrong set!");
                }



                //Checking rest battery capacity for each user
                double restBatCap = x[um.restEnergyVarInd];
                if (restBatCap < -p.doublePrecision)
                {
                    Misc.errOut("User " + um.u._userID + " has rest bat capacity in cplex solution lower than zero!");
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
                //Checking departure time for each user excpet ending depot which actually does not have a departure time
                if (!md.endingDepots.Contains(um))
                {
                    int count = 0;
                    double departureTime = 0;
                    foreach (KeyValuePair<(UserMILP, UserMILP, int), int> pair in md.Zetaijk)
                    {
                        if (x[pair.Value] > p.doublePrecision && pair.Key.Item1 == um)
                        {
                            if (x[md.Xijk[(pair.Key.Item1, pair.Key.Item2, pair.Key.Item3)]] > p.doublePrecision)
                            {
                                count++;
                                departureTime += x[pair.Value];
                            }

                        }
                    }
                    //If departure time is used for a user is used multiple times
                    // this would mean that he starts several times, and this was a big question
                    // did i model this part okay or not, as it has to be used only once per user (in all time periods)                    
                    if (count > 1)
                    {
                        Misc.errOut("Departure time variable is used multiple times!");
                    }
                    //For customer the departure time has to be equal to begintime+serviceTime
                    double computedDepartureTime = beginTime;
                    if (md.getV().Contains(um) || um == md.depot0)
                    {
                        computedDepartureTime += um.u.serviceTime;
                    }
                    else
                    {
                        Misc.errOut("Wrong set!");
                    }
                    //Compare departure times
                    if (Math.Abs(departureTime - computedDepartureTime) > p.doublePrecision)
                    {
                        Misc.errOut("Departure times in cplex solution do not match!");
                    }
                }
            }
            //Often when double resolution is lower 10^-7 the xijk value has a zero, or near zero value (10^-8).
            //The problem occurs when the value is 10^-8, then corresponding Tijk value can have a valeu larger than zero, i.e. 10^-5
            //which lead to errors when comparing extracted solution and cplex solution, thats why we decreased the double resoltion to 10^-3
            foreach (KeyValuePair<(UserMILP, UserMILP, int), int> pair in md.Xijk)
            {
                if (x[pair.Value] <= p.doublePrecision)
                {
                    int indextijkVal = md.Zetaijk[(pair.Key.Item1, pair.Key.Item2, pair.Key.Item3)];
                    if (x[indextijkVal] > p.doublePrecision)
                    {
                        Misc.errOut("The xijk values is zero, while the Zetaijk values is larger than zero!");
                    }
                }
            }
            //Compare number of arcs for ending depots
            if (numInputArcsEndDepots != numVehs)
            {
                Misc.errOut("Number of input arcs into ending depots differs from num of vehicles!");
            }





            //Station part
            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getV_ED())
                {
                    //Checking the number of CS between i and j
                    List<int> varISJIndices = md.getAllVarIndicesOfCSBetweenIJInZISJK(i, j);
                    int countLargerThen1 = 0;
                    foreach (int varIndex in varISJIndices)
                    {
                        if (x[varIndex] > p.doublePrecision)
                        {
                            countLargerThen1++;
                        }
                    }
                    if (countLargerThen1 > 1)
                    {
                        Misc.errOut("Should not be more than 1 CS between customers!");
                    }

                    foreach (UserMILP s in md.getF())
                    {
                        int countCheck = 0;
                        for (int k = 0; k < md.kBuckets; k++)
                        {
                            if (md.Zisjk.ContainsKey((i, s, j, k)))
                            {
                                int varIndexZisjk = md.Zisjk[(i, s, j, k)];
                                if (x[varIndexZisjk] > p.doublePrecision)
                                {
                                    countCheck++;
                                    int varIndexYisj = md.Yisj[(i, s, j)];
                                    //Checking rest battery capacity for CS
                                    double restBatCap = x[varIndexYisj];
                                    if (restBatCap < -p.doublePrecision)
                                    {
                                        Misc.errOut("CS " + s.u._userID + " has rest bat capacity in cplex solution lower than zero!");
                                    }
                                }
                                ////Checking departure time at CS
                                //int varIndexZisjk = md.Yisj[(i, s, j)];


                                //int count = 0;
                                //    double departureTime = 0;
                                //    foreach (KeyValuePair<(UserMILP, UserMILP, UserMILP, int), int> pair in md.Zetaisjk)
                                //    {
                                //        if (x[pair.Value] > p.doublePrecision && pair.Key.Item1 == um)
                                //        {
                                //            if (x[md.Xijk[(pair.Key.Item1, pair.Key.Item2, pair.Key.Item3)]] > p.doublePrecision)
                                //            {
                                //                count++;
                                //                departureTime += x[pair.Value];
                                //            }

                                //        }
                                //    }
                                //    //If departure time is used for a user is used multiple times
                                //    // this would mean that he starts several times, and this was a big question
                                //    // did i model this part okay or not, as it has to be used only once per user (in all time periods)                    
                                //    if (count > 1)
                                //    {
                                //        Misc.errOut("Departure time variable is used multiple times!");
                                //    }
                                //    //For customer the departure time has to be equal to begintime+serviceTime
                                //    double computedDepartureTime = beginTime;
                                //    if (md.getV().Contains(um) || um == md.depot0)
                                //    {
                                //        computedDepartureTime += um.u.serviceTime;
                                //    }
                                //    else
                                //    {
                                //        Misc.errOut("Wrong set!");
                                //    }
                                //    //Compare departure times
                                //    if (Math.Abs(departureTime - computedDepartureTime) > p.doublePrecision)
                                //    {
                                //        Misc.errOut("Departure times in cplex solution do not match!");
                                //    }

                            }
                        }
                        if (countCheck > 1)
                        {
                            Misc.errOut("Too many active zijsk for fixed i,s and j!");
                        }
                    }

                }
            }
        }








        /*Returns instance of solution extracted from cplex solution array x=A^{-1}b*/
        private Solution getSolutionFromCplexOld(MILPDataOld md, double[] x)
        {
            //Create instance of solution
            Solution s = new Solution(p);
            int numVeh = 0;
            foreach (KeyValuePair<(UserMILP, UserMILP, int), int> pair in md.Xijk)
            {
                if (x[pair.Value] > p.doublePrecision && pair.Key.Item1 == md.depot0)
                {
                    numVeh++;
                    //New vehicle
                    Vehicle v = new Vehicle(p, s.vehicles.Count, s);
                    //Arc 0->j
                    UserMILP ui = pair.Key.Item1;
                    UserMILP uj = pair.Key.Item2;
                    while (true)
                    {
                        //If ending depot is found, close vehicle and update all the vehicle and solotion values
                        if (md.endingDepots.Contains(uj))
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

                            //Check begin times
                            double beginTimeCplex = x[uj.serviceStartTimeVarInd];
                            if (!Misc.EqualDoubleValues(beginTimeCplex, uj.u.beginTime, p.doublePrecision))
                            {
                                Misc.errOut("Begin times do not coincide!");
                            }
                            //Check rest battery capacity
                            double restBatCap = x[uj.restEnergyVarInd];
                            if (!Misc.EqualDoubleValues(restBatCap, uj.u.restBatCapAtArrival, p.doublePrecision))
                            {
                                Misc.errOut("Rest bat capacity at arrival do not coincide!");
                            }
                            //Checking departure time for each user excpet ending depot which actually does not have a departure time
                            if (!md.endingDepots.Contains(uj))
                            {
                                int count = 0;
                                double departureTimeCplex = 0;
                                foreach (KeyValuePair<(UserMILP, UserMILP, int), int> pair3 in md.Tijk)
                                {
                                    if (x[pair3.Value] > p.doublePrecision && pair3.Key.Item1 == uj)
                                    {
                                        count++;
                                        departureTimeCplex += x[pair3.Value];
                                    }
                                }
                                //If departure time is used for a user is used multiple times
                                // this would mean that he starts several times, and this was a big question
                                // did i model this part okay or not, as it has to be used only once per user (in all time periods)
                                if (count > 1)
                                {
                                    Misc.errOut("Departure time variable is used multiple times!");
                                }
                                //For customer the departure time has to be equal to begintime+serviceTim
                                if (!Misc.EqualDoubleValues(departureTimeCplex, uj.u.departureTime, p.doublePrecision))
                                {
                                    Misc.errOut("Departure times do not coincide!");
                                }
                            }
                            foreach (KeyValuePair<(UserMILP, UserMILP, int), int> pair2 in md.Xijk)
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


        private Solution getSolutionFromCplexNew(MILPDataNew md, double[] x)
        {
            //Create instance of solution
            Solution s = new Solution(p);
            int numVeh = 0;

            int cntNumCS = 0;
            foreach (KeyValuePair<(UserMILP, UserMILP, UserMILP, int), int> pairZISJK in md.Zisjk)
            {

                if (x[pairZISJK.Value] > p.doublePrecision)
                {
                    cntNumCS++;
                    Console.WriteLine($"{pairZISJK.Key.Item1.u._userID}->{pairZISJK.Key.Item2.u._userID}->{pairZISJK.Key.Item3.u._userID}");

                }
            }

            foreach (KeyValuePair<(UserMILP, UserMILP, int), int> pair in md.Xijk)
            {
                if (x[pair.Value] > p.doublePrecision && pair.Key.Item1 == md.depot0)
                {
                    numVeh++;
                    //New vehicle
                    Vehicle v = new Vehicle(p, s.vehicles.Count, s);
                    //Arc 0->j
                    UserMILP ui = pair.Key.Item1;
                    UserMILP uj = pair.Key.Item2;
                    while (true)
                    {
                        int cntCS = 0;
                        helpVar++;
                        if (helpVar == 5)
                        {
                            Console.WriteLine();
                        }
                        //First check if station is traversed
                        int count = 0;
                        UserMILP CS = null;
                        User CSInRoute = null;
                        double chargingTime = 0;
                        int kZisjk = -1;
                        User iu = v.route.Last();
                        foreach (KeyValuePair<(UserMILP, UserMILP, UserMILP , int), int> pairZISJK in md.Zisjk)
                        {
                            if (pairZISJK.Key.Item1 == ui && pairZISJK.Key.Item3 == uj)
                            {
                                if (x[pairZISJK.Value] > p.doublePrecision)
                                {
                                    count++;
                                    CS = pairZISJK.Key.Item2;
                                    kZisjk = pairZISJK.Key.Item4;
                                    cntCS++;
                                }
                            }
                        }
                        if (count > 1)
                        {
                            Misc.errOut("Cannot be more than 1 CS in Zisjk!");
                        }

                        if (CS != null)
                        {
                            CSInRoute = new User(CS.u._userID, CS.u.x, CS.u.y, CS.u.demand, CS.u.etw, CS.u.ltw, CS.u.serviceTime, CS.u.type, CS.u.p);
                            v.route.Add(CSInRoute);
                        }
                        v.route.Add(uj.u);
                        v.updateVehicle();

                        User ju = v.route.Last();

                        //Check xijk
                        int kXIJK = -1;
                        foreach (KeyValuePair<(UserMILP, UserMILP, int), int> pairXijk in md.Xijk)
                        {
                            if (pairXijk.Key.Item1 == ui && pairXijk.Key.Item2 == uj)
                            {
                                if (x[pairXijk.Value] > p.doublePrecision)
                                {
                                    kXIJK = pairXijk.Key.Item3;
                                }
                            }
                        }
                        if (count > 1)
                        {
                            Misc.errOut("Cannot be more than 1 CS in Zisjk!");
                        }

                        //Check begin times at I
                        double beginTimeICplex = x[ui.serviceStartTimeVarInd];
                        if (!Misc.EqualDoubleValues(beginTimeICplex, iu.beginTime, p.doublePrecision))
                        {
                            Misc.errOut("Begin times do not coincide!");
                        }
                        //Check begin times at J
                        double beginTimeJCplex = x[uj.serviceStartTimeVarInd];
                        if (!Misc.EqualDoubleValues(beginTimeJCplex, ju.beginTime, p.doublePrecision))
                        {
                            Misc.errOut("Begin times do not coincide!");
                        }

                        //Check departure time at I
                        double departureTimeAtI = 0;
                        int kZetaijk = 0;
                        count = 0;
                        foreach (KeyValuePair<(UserMILP, UserMILP, int), int> pairZetaIJK in md.Zetaijk)
                        {
                            if (pairZetaIJK.Key.Item1 == ui && pairZetaIJK.Key.Item2 == uj)
                            {
                                if (x[pairZetaIJK.Value] > p.doublePrecision)
                                {
                                    count++;
                                    departureTimeAtI += x[pairZetaIJK.Value];
                                    kZetaijk = pairZetaIJK.Key.Item3;
                                }
                            }
                        }
                        if (count > 1)
                        {
                            Misc.errOut("Cannot be more than 1 ZetaIJK !");
                        }
                        if (!Misc.EqualDoubleValues(departureTimeAtI, iu.departureTime, p.doublePrecision))
                        {
                            Misc.errOut("Departure times at i do not coincide!");
                        }
                        if (kZetaijk != p.getTimeIntervalIndex(iu.departureTime) && count > 0)
                        {
                            Console.WriteLine($"Departure time periods kZetaijk={kZetaijk} and comp dep time ({p.getTimeIntervalIndex(iu.departureTime)})(k) at i do not coincide!");
                        }
                        if (kXIJK != kZetaijk && count > 0)
                        {
                            Misc.errOut("kXIJK and kZetaijk time periods (k) at i do not coincide!");
                        }

                        //Check rest battery capacity at I
                        double restBatCapatI = x[ui.restEnergyVarInd];
                        if (!Misc.EqualDoubleValues(restBatCapatI, iu.restBatCapAtArrival, p.doublePrecision))
                        {
                            Misc.errOut("Rest bat capacity at arrival do not coincide!");
                        }

                        //Check rest battery capacity at J
                        double restBatCapatJ = x[uj.restEnergyVarInd];
                        if (!Misc.EqualDoubleValues(restBatCapatJ, ju.restBatCapAtArrival, p.doublePrecision))
                        {
                            Misc.errOut("Rest bat capacity at arrival do not coincide!");
                        }

                        double DeltaISJ = 0;

                        if (CS == null)
                        {
                            foreach (UserMILP cs in md.getF())
                            {
                                if (md.Deltaisj.ContainsKey((ui, cs, uj)))
                                {
                                    DeltaISJ += x[md.Deltaisj[(ui, cs, uj)]];
                                }
                            }
                            if (!Misc.EqualDoubleValues(DeltaISJ, 0, p.doublePrecision))
                            {
                                Misc.errOut("DeltaISJ should be zero and it is not!");
                            }
                        }
                        //User CSInroute = null;

                        if (CS != null)
                        {
                            //Check k
                            if (kZisjk != kXIJK)
                            {
                                Misc.errOut("kZisjk and kXIJK time periods (k) at i do not coincide!");
                            }

                            //Check k
                            if (kZisjk != kZetaijk)
                            {
                                Misc.errOut("Zisjk and kZetaijk time periods (k) at i do not coincide!");
                            }


                            //Check rest battery capacity
                            int varIndex = md.Yisj[(ui, CS, uj)];
                            double restBatCapAtCS = x[varIndex];
                            if (!Misc.EqualDoubleValues(restBatCapAtCS, CSInRoute.restBatCapAtArrival, p.doublePrecision))
                            {
                                Misc.errOut("Rest bat capacity at arrival do not coincide in CS!");
                            }
                            chargingTime = p.refuelRate * (p.batCap - restBatCapAtCS);

                            //For customer the departure time has to be equal to begintime+serviceTim
                            if (!Misc.EqualDoubleValues(chargingTime, CSInRoute.serviceTime, p.doublePrecision))
                            {
                                Misc.errOut("Charging times do not coincide!");
                            }

                            //Check zisjk
                            double zisjk = x[md.Zisjk[(ui, CS, uj, kZisjk)]];
                            if (zisjk <= p.doublePrecision)
                            {
                                Console.WriteLine("Something wrong wiht zisjk!");
                            }

                            //Check begin time at CS
                            double beginTimeAtCSCplex = departureTimeAtI + p.getSlopeFromLinTime(ui.u, CS.u, kZetaijk) * departureTimeAtI +
                                p.getSectionFromLinTime(ui.u, CS.u, kZetaijk) * zisjk;
                            if (!Misc.EqualDoubleValues(beginTimeAtCSCplex, CSInRoute.beginTime, p.doublePrecision))
                            {
                                Misc.errOut("Begin times at CS do not coincide!");
                            }

                            //Departure time at CS
                            count = 0;
                            double departureTimeAtCS = 0;
                            int kZetaisjk = -1;
                            foreach (KeyValuePair<(UserMILP, UserMILP, UserMILP, int), int> pair3 in md.Zetaisjk)
                            {
                                if (x[pair3.Value] > p.doublePrecision && pair3.Key.Item1 == ui && pair3.Key.Item2 == CS && pair3.Key.Item3 == uj)
                                {
                                    count++;
                                    departureTimeAtCS += x[pair3.Value];
                                    kZetaisjk = pair3.Key.Item4;
                                }
                            }
                            //If departure time is used for a user is used multiple times
                            // this would mean that he starts several times, and this was a big question
                            // did i model this part okay or not, as it has to be used only once per user (in all time periods)
                            if (count > 1)
                            {
                                Misc.errOut("Departure time variable is used multiple times!");
                            }

                            if (kZetaisjk != kZisjk)
                            {
                                Console.WriteLine("This should be allowed sometimes!");
                            }

                            //For customer the departure time has to be equal to begintime+serviceTim
                            if (!Misc.EqualDoubleValues(departureTimeAtCS, CSInRoute.departureTime, p.doublePrecision))
                            {
                                Misc.errOut("Departure times do not coincide!");
                            }

                            count = 0;
                            int kZ_isjk = -1;
                            foreach (KeyValuePair<(UserMILP, UserMILP, UserMILP, int), int> pair3 in md.Z_isjk)
                            {
                                if (x[pair3.Value] > p.doublePrecision && pair3.Key.Item1 == ui && pair3.Key.Item2 == CS && pair3.Key.Item3 == uj)
                                {
                                    count++;
                                    kZ_isjk = pair3.Key.Item4;
                                }
                            }

                            if (count != 1)
                            {
                                Misc.errOut("Z_isjk  variable is used multiple times or none at all!");
                            }

                            if (kZetaisjk != kZ_isjk)
                            {
                                Misc.errOut("kZetaisjk and kZ_isjk time periods (k) at i do not coincide!!");
                            }

                            count = 0;
                            DeltaISJ = 0;
                            foreach (UserMILP cs in md.getF())
                            {
                                if (md.Deltaisj.ContainsKey((ui, cs, uj)))
                                {
                                    if (x[md.Deltaisj[(ui, cs, uj)]] > p.doublePrecision)
                                    {
                                        DeltaISJ += x[md.Deltaisj[(ui, cs, uj)]];
                                        count++;
                                    }
                                }

                            }

                            if (count != 1)
                            {
                                Misc.errOut("Deltaisj  variable is used multiple times!");
                            }
                            double compAdditionalTime = departureTimeAtCS - departureTimeAtI +
                                departureTimeAtCS * p.getSlopeFromLinTime(CS.u, uj.u, kZ_isjk) +
                                count * p.getSectionFromLinTime(CS.u, uj.u, kZ_isjk) -
                                (departureTimeAtI * p.getSlopeFromLinTime(ui.u, uj.u, kZisjk) +
                                count * p.getSectionFromLinTime(ui.u, uj.u, kZisjk));


                            if (!Misc.EqualDoubleValues(DeltaISJ, compAdditionalTime, p.doublePrecision))
                            {
                                Misc.errOut("DeltaISJ should be equal and it is not!");
                            }

                        }

                        //Checking departure time for each user excpet ending depot which actually does not have a departure time
                        if (!md.endingDepots.Contains(uj))
                        {
                            count = 0;
                            double departureTimeJCplex = 0;
                            foreach (KeyValuePair<(UserMILP, UserMILP, int), int> pair3 in md.Zetaijk)
                            {
                                if (x[pair3.Value] > p.doublePrecision && pair3.Key.Item1 == uj)
                                {
                                    count++;
                                    departureTimeJCplex += x[pair3.Value];
                                }
                            }
                            if (count > 1)
                            {
                                Misc.errOut("Departure time variable at j is used multiple times!");
                            }
                            //For customer the departure time has to be equal to begintime+serviceTim
                            if (!Misc.EqualDoubleValues(departureTimeJCplex, ju.departureTime, p.doublePrecision))
                            {
                                Misc.errOut("Departure times at j do not coincide!");
                            }


                        }


                        //If ending depot is found, close vehicle and update all the vehicle and solotion values
                        if (md.endingDepots.Contains(uj))
                        {
                            //v.AddDepotToEnd();
                            v.updateVehicle();
                            s.addVehicle(v);
                            if (cntCS > 1)
                            {
                                Misc.errOut("It exists");
                            }
                            Console.WriteLine($"NUm of CSs:{cntCS}");
                            break;
                        }



                        foreach (KeyValuePair<(UserMILP, UserMILP, int), int> pair2 in md.Xijk)
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
            return s;
        }
    }
}

