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

        internal TerminateCallback(double lastIncumbent,int bestTheoryMin)
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

            Console.WriteLine("Solving TDEVRPTW with full recharge: " + p.instanceName + ", num customers=" + p.customers.Count +
                ", num virtual CSs=" + p.numMultiCSVert + ", knownMinVehNum=" + p.knownVehNumberCPLEX + ", minimization objective: " + p.minimizationType.ToString() +
                ", Fig Coeffs:" + p.travelTimeCompType);

            //Create an instance of MILP graph data: vertices, variables etc.
            //int numEndingDepots = Convert.ToInt32(Math.Ceiling(p.theoryMinNumVehicle * 1.4));
            int numEndingDepots = 5;
            md = new MILPData(p,numEndingDepots);
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
                md = new MILPData(p, numVehicles);
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
            foreach (KeyValuePair<(UserMILP, UserMILP, int), int> element in md.Xijk)
            {
                int varIndexXijk = element.Value;
                bm.xtb[varIndexXijk] = NumVarType.Int;
                bm.lb[varIndexXijk] = 0;
                bm.ub[varIndexXijk] = 1;
            }

            //TIJK - float variable actually representign departure time
            foreach (KeyValuePair<(UserMILP, UserMILP, int), int> element in md.Tijk)
            {
                int varIndexTijk = element.Value;
                bm.xtb[varIndexTijk] = NumVarType.Float;
                //Ovo je bila greška što nije bilo 0, mora biti 0 jer u u slučajevima u kojima varijabla t_{ij}
                // nije aktivna jer jer xij=0, i ona mora biti 
                bm.lb[varIndexTijk] = 0;

                UserMILP u = element.Key.Item1;
                if (md.getV0().Contains(u))
                {
                    bm.ub[varIndexTijk] = u.u.ltw + u.u.serviceTime;
                }
                else if (md.getF_().Contains(u))
                {
                    bm.ub[varIndexTijk] = u.u.ltw + p.refuelRate * p.batCap;
                }
                else
                {
                    Misc.errOut("Error! User is not ni F' or V0!");
                }
            }

            //BIJ - binary decision variable (0,1) integer
            foreach (KeyValuePair<(UserMILP, UserMILP), int> element in md.Bij)
            {
                int varIndexBij = element.Value;
                bm.xtb[varIndexBij] = NumVarType.Int;
                bm.lb[varIndexBij] = 0;
                bm.ub[varIndexBij] = 1;
            }

            //Users
            foreach (UserMILP um in md.getV0Nm_())
            {
                //Service start time windows bound
                int varIndexTWStart = um.serviceStartTimeVarInd;
                bm.xtb[varIndexTWStart] = NumVarType.Float;
                bm.lb[varIndexTWStart] = um.u.etw;
                bm.ub[varIndexTWStart] = um == md.depot0 ?um.u.etw: um.u.ltw;

                //Load capacity bound
                int varIndexLoad = um.restLoadVarInd;
                bm.xtb[varIndexLoad] = NumVarType.Float;
                bm.lb[varIndexLoad] = um == md.depot0 ? p.loadCap : 0;
                bm.ub[varIndexLoad] = p.loadCap;

                //Energy capacity bound
                int varIndexEnergy = um.restEnergyVarInd;
                bm.xtb[varIndexEnergy] = NumVarType.Float;
                bm.lb[varIndexEnergy] = (um == md.depot0) ? p.batCap : 0;
                bm.ub[varIndexEnergy] = p.batCap;
            }

            //Equations Ax=b

            /*Only 1 exit arc from eac customer in all time buckets
             *              * */
            //tex: Equation $\forall i \in V$ $\sum_{k \in K} \sum_{j \in V_{N+1} \cup F', i \neq j} x_{ij}^k=1$ 
            foreach (UserMILP i in md.getV())
            {
                double[] row = new double[md.numVars];
                foreach (UserMILP j in md.getVNm_())
                {
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        if (md.Xijk.ContainsKey((i, j, k)))
                        {
                            row[md.Xijk[(i, j, k)]] = 1;
                        }
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

            /*
             * At max 1 exit arc from each CS (some could have zero) in all time buckets
             */
            //tex: $\forall i \in F'$ $\sum_{k \in K} \sum_{j \in V_{N+1} \cup F', i \neq j} x_{ij}^k \leq 1$ 
            foreach (UserMILP i in md.getF_())
            {
                double[] row = new double[md.numVars];
                foreach (UserMILP j in md.getVNm_())
                {
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        if (md.Xijk.ContainsKey((i, j, k)))
                        {
                            row[md.Xijk[(i, j, k)]] = 1;
                        }

                    }
                }
                if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                {
                    Misc.errOut("All values in a row should not be zero!");
                }
                bm.A.Add(row);
                bm.b.Add(1);
                bm.eqType.Add("lowerOrEqual");
            }


            // Flow conservation-> number of exit arcs has to be equal to the number of entry arcs for all arcs in all time periods
            //tex: $\forall j \in V \cup F'$ $\sum_{k \in K} \sum_{i \in V_{N+1} \cup F', i \neq j} x_{ji}^k - \sum_{k \in K} \sum_{i \in V_{0} \cup F', i \neq j} x_{ij}^k = 0$ 
            foreach (UserMILP j in md.getV_())
            {
                double[] row = new double[md.numVars];
                for (int k = 0; k < md.kBuckets; k++)
                {
                    foreach (UserMILP i in md.getVNm_())
                    {
                        if (md.Xijk.ContainsKey((j, i, k)))
                        {
                            row[md.Xijk[(j, i, k)]] += 1;
                        }
                    }
                    foreach (UserMILP i in md.getV0_())
                    {
                        if (md.Xijk.ContainsKey((i, j, k)))
                        {
                            row[md.Xijk[(i, j, k)]] -= 1;
                        }
                    }
                }
                if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                {
                    Misc.errOut("All values in a row should not be zero!");
                }
                bm.A.Add(row);
                bm.b.Add(0);
                bm.eqType.Add("equal");
            }

            /*Only 1 entry arc for each ending depot
 *              * */
            //tex: Equation $\forall j \in MED$ $\sum_{k \in K} \sum_{i \in V \cup F', i \neq j} x_{ij}^k=1$ 
            foreach (UserMILP j in md.getMED())
            {
                double[] row = new double[md.numVars];
                foreach (UserMILP i in md.getV_())
                {
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        if (md.Xijk.ContainsKey((i, j, k)))
                        {
                            row[md.Xijk[(i, j, k)]] = 1;
                        }
                    }
                }
                if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                {
                    Misc.errOut("All values in a row should not be zero!");
                }
                bm.A.Add(row);
                bm.b.Add(1);
                bm.eqType.Add("lowerOrEqual");
            }


            // Equation for the computation of service start time at customer (probably can be ommited in future versions)
            //tex: $\forall i \in V_{0} $ $t_i-\sum_{k \in K} \sum_{j \in V_{N+1} \cup F', i \neq j} t_{ij}^k=-s_i$ 
            //
            //Provjeriti
            foreach (UserMILP i in md.getV0())
            {
                double[] row = new double[md.numVars];
                for (int k = 0; k < md.kBuckets; k++)
                {
                    foreach (UserMILP j in md.getVNm_())
                    {
                        if (md.Tijk.ContainsKey((i, j, k)))
                        {
                            row[md.Tijk[(i, j, k)]] = -1;
                        }
                    }
                }
                row[i.serviceStartTimeVarInd] = 1;
                if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                {
                    Misc.errOut("All values in a row should not be zero!");
                }
                bm.A.Add(row);
                bm.b.Add(-i.u.serviceTime);
                bm.eqType.Add("equal");
            }

            // Equation for the computation of service start time at CS (probably can be ommited in future versions)
            //tex: $\forall i \in F' $ $t_i-\sum_{k \in K} \sum_{j \in V_{N+1} \cup F', i \neq j} t_{ij}^k-gy_i=-gQ$ 

            //Provjeriti
            foreach (UserMILP i in md.getF_())
            {
                double[] row = new double[md.numVars];
                for (int k = 0; k < md.kBuckets; k++)
                {
                    foreach (UserMILP j in md.getVNm_())
                    {
                        if (md.Tijk.ContainsKey((i, j, k)))
                        {
                            row[md.Tijk[(i, j, k)]] = -1;
                        }
                    }
                }
                row[i.serviceStartTimeVarInd] = 1;
                row[i.restEnergyVarInd] = -p.refuelRate;
                if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                {
                    Misc.errOut("All values in a row should not be zero!");
                }
                bm.A.Add(row);
                bm.b.Add(-p.refuelRate * p.batCap);
                bm.eqType.Add("equal");
            }

            // Equation for time feasibility for arcs leaving customers and depot
            //tex: $\forall i \in V_0, \forall j \in V_{N+1} \cup F'~ i\neq j $ $t_i+s_i \sum_{k \in K}x_{ij}^k+\sum_{k \in K} (\Theta_{ij}^k t_{ij}^k + \eta_{ij}^k x_{ij}^k) - (l_0+gQ)(1-\sum_{k \in K} x_{ij}^k) \leq t_j$ 

            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getVNm_())
                {
                    double[] row = new double[md.numVars];
                    bool any = false;
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        if (md.Tijk.ContainsKey((i, j, k)) && md.Xijk.ContainsKey((i, j, k)))
                        {
                            any = true;

                            row[md.Tijk[(i, j, k)]] = p.getSlopeFromLinTime(i.u, j.u, k);
                            row[md.Xijk[(i, j, k)]] = p.getSectionFromLinTime(i.u, j.u, k) + p.depot.ltw+p.refuelRate*p.batCap + i.u.serviceTime;
                        }
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
                        bm.b.Add(p.depot.ltw+p.refuelRate*p.batCap);
                        bm.eqType.Add("lowerOrEqual");
                    }
                }
            }

            // Equation for time feasibility for arcs leaving customers and depot max fun on begin time -tj
            //tex: $\forall i \in V_0, \forall j \in V_{N+1} \cup F'~ i\neq j $ $t_i+s_i \sum_{k \in K}x_{ij}^k+\sum_{k \in K} (\Theta_{ij}^k t_{ij}^k + \eta_{ij}^k x_{ij}^k) - (l_0+gQ)(1-\sum_{k \in K} x_{ij}^k) +2(l_0+gQ)b_j \geq t_j$ 

            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getVNm_())
                {
                    double[] row = new double[md.numVars];
                    bool any = false;
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        if (md.Tijk.ContainsKey((i, j, k)) && md.Xijk.ContainsKey((i, j, k)))
                        {
                            any = true;

                            row[md.Tijk[(i, j, k)]] = p.getSlopeFromLinTime(i.u, j.u, k);
                            row[md.Xijk[(i, j, k)]] = p.getSectionFromLinTime(i.u, j.u, k) + p.depot.ltw+ p.refuelRate * p.batCap + i.u.serviceTime;
                        }
                    }
                    if (any)
                    {
                        row[i.serviceStartTimeVarInd] = 1;
                        row[j.serviceStartTimeVarInd] = -1;
                        row[md.Bij[(i, j)]] =2*(p.depot.ltw+ p.refuelRate * p.batCap);
                        if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                        {
                            Misc.errOut("All values in a row should not be zero!");
                        }
                        bm.A.Add(row);
                        bm.b.Add(p.depot.ltw+ p.refuelRate * p.batCap);
                        bm.eqType.Add("greaterOrEqual");
                    }
                }
            }


            // Equation for time feasibility for arcs leaving customers and depot max fun on begin time - ej
            //tex: $\forall i \in V_0, \forall j \in V_{N+1} \cup F'~ i\neq j $ $t_j \leq e_j+(1-b_j)2(l_0+gQ)+b_j(l_0+gQ) -\sum_{k \in K}x_{ij}^k(l_0+gQ)$ 

            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getVNm_())
                {
                    double[] row = new double[md.numVars];
                    bool any = false;
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        if (md.Xijk.ContainsKey((i, j, k)))
                        {
                            any = true;
                            row[md.Xijk[(i, j, k)]] = p.depot.ltw+ p.refuelRate * p.batCap;
                        }
                    }
                    if (any)
                    {
                        row[j.serviceStartTimeVarInd] = 1;
                        row[md.Bij[(i, j)]] = (p.depot.ltw+ p.refuelRate * p.batCap);
                        if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                        {
                            Misc.errOut("All values in a row should not be zero!");
                        }
                        bm.A.Add(row);
                        bm.b.Add(j.u.etw+2*(p.depot.ltw+ p.refuelRate * p.batCap));
                        bm.eqType.Add("lowerOrEqual");
                    }
                }
            }


            // Equation for time feasibility for arcs leaving CSs
            //tex: $\forall i \in F', \forall j \in V_{N+1} \cup F'~ i\neq j$ $t_i+g(Q-y_i)+\sum_{k \in K} (\Theta_{ij}^k t_{ij}^k + \eta_{ij}^k x_{ij}^k) - (l_0+gQ)(1-\sum_{k \in K}x_{ij}^k) \leq t_j$ 

            foreach (UserMILP i in md.getF_())
            {
                foreach (UserMILP j in md.getVNm_())
                {
                    double[] row = new double[md.numVars];
                    bool any = false;
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        if (md.Tijk.ContainsKey((i, j, k)) && md.Xijk.ContainsKey((i, j, k)))
                        {
                            any = true;
                            row[md.Tijk[(i, j, k)]] = p.getSlopeFromLinTime(i.u, j.u, k);
                            row[md.Xijk[(i, j, k)]] = p.getSectionFromLinTime(i.u, j.u, k) + p.depot.ltw + p.refuelRate * p.batCap;
                        }
                    }
                    if (any)
                    {
                        row[i.serviceStartTimeVarInd] = 1;
                        row[j.serviceStartTimeVarInd] = -1;
                        row[i.restEnergyVarInd] = -p.refuelRate;
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

            // Equation for time feasibility for arcs leaving CSs - max fun begin time - tj
            //tex: $\forall i \in F', \forall j \in V_{N+1} \cup F'~ i\neq j$ $t_i+g(Q-y_i)+\sum_{k \in K} (\Theta_{ij}^k t_{ij}^k + \eta_{ij}^k x_{ij}^k) - (l_0+gQ)(1-\sum_{k \in K}x_{ij}^k)+2b_j(l_0+gQ) \geq t_j$ 

            foreach (UserMILP i in md.getF_())
            {
                foreach (UserMILP j in md.getVNm_())
                {
                    double[] row = new double[md.numVars];
                    bool any = false;
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        if (md.Tijk.ContainsKey((i, j, k)) && md.Xijk.ContainsKey((i, j, k)))
                        {
                            any = true;
                            row[md.Tijk[(i, j, k)]] = p.getSlopeFromLinTime(i.u, j.u, k);
                            row[md.Xijk[(i, j, k)]] = p.getSectionFromLinTime(i.u, j.u, k) + p.depot.ltw + p.refuelRate * p.batCap;
                        }
                    }
                    if (any)
                    {
                        row[i.serviceStartTimeVarInd] = 1;
                        row[j.serviceStartTimeVarInd] = -1;
                        row[i.restEnergyVarInd] = -p.refuelRate;
                        row[md.Bij[(i, j)]] = 2*(p.refuelRate*p.batCap+p.depot.ltw);
                        if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                        {
                            Misc.errOut("All values in a row should not be zero!");
                        }
                        bm.A.Add(row);
                        bm.b.Add(p.depot.ltw);
                        bm.eqType.Add("greaterOrEqual");
                    }
                }
            }

            // Equation for time feasibility for arcs leaving CSs - max fun begin time - etw
            //tex: $\forall i \in F', \forall j \in V_{N+1} \cup F'~ i\neq j$ $t_j \leq e_j +(1-b)2(l_0+gQ)+b(l_0+gQ)-\sum_{k \in K}x_{ij}^k(l_0+gQ)$ 

            foreach (UserMILP i in md.getF_())
            {
                foreach (UserMILP j in md.getVNm_())
                {
                    double[] row = new double[md.numVars];
                    bool any = false;
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        if (md.Xijk.ContainsKey((i, j, k)))
                        {
                            any = true;
                            row[md.Xijk[(i, j, k)]] = p.depot.ltw+p.refuelRate*p.batCap;
                        }
                    }
                    if (any)
                    {
                        row[j.serviceStartTimeVarInd] = 1;
                        row[md.Bij[(i, j)]] = p.depot.ltw+p.refuelRate*p.batCap;
                        if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                        {
                            Misc.errOut("All values in a row should not be zero!");
                        }
                        bm.A.Add(row);
                        bm.b.Add(j.u.etw+2*(p.depot.ltw+p.refuelRate*p.batCap));
                        bm.eqType.Add("lowerOrEqual");
                    }
                }
            }

            // Equation for ensuring that each departure time of customer/CS is within its appropriate breakpointes
            // determined by breakpoints of time buckets and latest departure times at customers/CSs
            // For customers the search space is adequatly reduced, while for CSs this is not the case
            //tex: $\forall i \in V_0 \cup F', \forall j \in V_{N+1} \cup F'~ i\neq j, \forall k \in K$ $x_{ij}^k \max{ (w_m,e_i+s_i)} \leq t_{ij}^k \leq \min{ (w_{m+1},l_i+s_i)}x_{ij}^k $ 

            foreach (UserMILP i in md.getV0_())
            {
                foreach (UserMILP j in md.getVNm_())
                {
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        if (md.Tijk.ContainsKey((i, j, k)) && md.Xijk.ContainsKey((i, j, k)))
                        {
                            double[] row1 = new double[md.numVars];
                            Tuple<double, double> boundaries = p.getBoundaries(k);
                            row1[md.Xijk[(i, j, k)]] = Math.Max(boundaries.Item1, i.u.etw + i.u.serviceTime);
                            row1[md.Tijk[(i, j, k)]] = -1;
                            if (!row1.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                            {
                                Misc.errOut("All values in a row should not be zero!");
                            }
                            bm.A.Add(row1);
                            bm.b.Add(0);
                            bm.eqType.Add("lowerOrEqual");

                            double[] row2 = new double[md.numVars];
                            if (md.getV0().Contains(i))
                            {
                                row2[md.Xijk[(i, j, k)]] = -Math.Min(boundaries.Item2, i.u.ltw + i.u.serviceTime);
                            }
                            else if (md.getF_().Contains(i))
                            {
                                row2[md.Xijk[(i, j, k)]] = -Math.Min(boundaries.Item2, i.u.ltw + p.refuelRate * p.batCap);
                            }
                            else
                            {
                                Misc.errOut("Not in either of sets F_ or V0!");
                            }
                            row2[md.Tijk[(i, j, k)]] = 1;
                            if (!row2.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                            {
                                Misc.errOut("All values in a row should not be zero!");
                            }
                            bm.A.Add(row2);
                            bm.b.Add(0);
                            bm.eqType.Add("lowerOrEqual");
                        }
                    }
                }
            }


            // Equation for reamining load capcity flow conservation
            //tex: $\forall i \in V_0 \cup F', \forall j \in V_{N+1} \cup F'~ i\neq j, u_j \leq u_i- \sum_{k \in K}x_{ij}^k (q_i+C)+C$ 

            foreach (UserMILP i in md.getV0_())
            {
                foreach (UserMILP j in md.getVNm_())
                {
                    double[] row = new double[md.numVars];
                    row[j.restLoadVarInd] = 1;
                    row[i.restLoadVarInd] = -1;
                    bool any = false;
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        if (md.Xijk.ContainsKey((i, j, k)))
                        {
                            row[md.Xijk[(i, j, k)]] = i.u.demand + p.loadCap;
                            any = true;
                        }
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

            // Equation for reamining battery capcity flow conservation (energy) for customers
            //tex: $\forall i \in V, \forall j \in V_{N+1} \cup F'~ i\neq j, y_j \leq y_i- \sum_{k \in K}x_{ij}^k (e_{ij}^k+Q)+Q$ 
            foreach (UserMILP j in md.getVNm_())
            {
                foreach (UserMILP i in md.getV())
                {
                    double[] row = new double[md.numVars];
                    row[j.restEnergyVarInd] = 1;
                    row[i.restEnergyVarInd] = -1;
                    bool any = false;
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        if (md.Xijk.ContainsKey((i, j, k)))
                        {
                            row[md.Xijk[(i, j, k)]] = p.ener(i.u, j.u, k) + p.batCap;
                            any = true;
                        }
                    }
                    if (any)
                    {
                        if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                        {
                            Misc.errOut("All values in a row should not be zero!");
                        }
                        bm.A.Add(row);
                        bm.b.Add(p.batCap);
                        bm.eqType.Add("lowerOrEqual");
                    }
                }
            }

            // Equation for reamining battery capcity flow conservation (energy) for customers - exact yj
            //Exact yj is necesearry only in TD cases when we jave to know the departure times
            //tex: $\forall i \in V, \forall j \in V_{N+1} \cup F'~ i\neq j, y_j \geq y_i- \sum_{k \in K}x_{ij}^k (e_{ij}^k-Q)-Q$ 
            foreach (UserMILP j in md.getVNm_())
            {
                foreach (UserMILP i in md.getV())
                {
                    double[] row = new double[md.numVars];
                    row[j.restEnergyVarInd] = 1;
                    row[i.restEnergyVarInd] = -1;
                    bool any = false;
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        if (md.Xijk.ContainsKey((i, j, k)))
                        {
                            row[md.Xijk[(i, j, k)]] = p.ener(i.u, j.u, k) - p.batCap;
                            any = true;
                        }
                    }
                    if (any)
                    {
                        if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                        {
                            Misc.errOut("All values in a row should not be zero!");
                        }
                        bm.A.Add(row);
                        bm.b.Add(-p.batCap);
                        bm.eqType.Add("greaterOrEqual");
                    }
                }
            }

            // Equation for reamining battery capcity flow conservation (energy) for customers
            //tex: $\forall i \in F'_0, \forall j \in V_{N+1} \cup F'~ i\neq j, y_j \leq Q- \sum_{k \in K}x_{ij}^k e_{ij}^k$ 

            foreach (UserMILP j in md.getVNm_())
            {

                foreach (UserMILP i in md.getF0_())
                {
                    double[] row = new double[md.numVars];
                    row[j.restEnergyVarInd] = 1;
                    bool any = false;
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        if (md.Xijk.ContainsKey((i, j, k)))
                        {
                            row[md.Xijk[(i, j, k)]] = p.ener(i.u, j.u, k);
                            any = true;
                        }
                    }
                    if (any)
                    {
                        if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                        {
                            Misc.errOut("All values in a row should not be zero!");
                        }
                        bm.A.Add(row);
                        bm.b.Add(p.batCap);
                        bm.eqType.Add("lowerOrEqual");
                    }
                }
            }

            // Equation for reamining battery capcity flow conservation (energy) for customers - exact yj
            //tex: $\forall i \in F'_0, \forall j \in V_{N+1} \cup F'~ i\neq j, y_j \geq \sum_{k \in K}x_{ij}^k (Q-e_{ij}^k)$ 

            foreach (UserMILP j in md.getVNm_())
            {

                foreach (UserMILP i in md.getF0_())
                {
                    double[] row = new double[md.numVars];
                    row[j.restEnergyVarInd] = 1;
                    bool any = false;
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        if (md.Xijk.ContainsKey((i, j, k)))
                        {
                            row[md.Xijk[(i, j, k)]] =-p.batCap+ p.ener(i.u, j.u, k);
                            any = true;
                        }
                    }
                    if (any)
                    {
                        if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                        {
                            Misc.errOut("All values in a row should not be zero!");
                        }
                        bm.A.Add(row);
                        bm.b.Add(0);
                        bm.eqType.Add("greaterOrEqual");
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
                //Minimize
                cplex.AddMinimize(cplex.ScalProd(var[0], objvalsVeh));
                //Set time limit
                cplex.SetParam(Cplex.Param.TimeLimit, p.timeLimitCplex);
                //It was noted that in some cases that the error out of memory occurs (especially on fast computer like toggrupa-01)
                //The default value is 10^75 MB, and here it is limited to a couple of GB (4-8)
                cplex.SetParam(Cplex.Param.MIP.Limits.TreeMemory, p.memoryLimitCplex);
                //In vehicle minimization we call a Terminate callbacke to terminate the optimization when minimal theory number is found
                cplex.Use(new TerminateCallback(double.MaxValue,p.theoryMinNumVehicle));
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
            // for entry arcs into a ending depot instances - i think this is actually not necesarry but not sure
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
                //tex: $\sum_{k \in K}\sum_{i \in V_0 \cup F'}  \sum_{j \in V_{N+1} \cup F'}x_{ij}^kd_{ij}^k$

                //Travel time
                //tex: $\sum_{k \in K}\sum_{i \in V_0 \cup F'}  \sum_{j \in V_{N+1} \cup F'}t_{ij}^k \Theta_{ij}^k +x_{ij}^k \mu_{ij}^k$

                //Total time - this equation should be okay in all cases
                //tex: $\sum_{k \in K}\sum_{i \in MED} t_{i}^k$

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
                    else if (p.minimizationType == MinimizationType.TravelTime)
                    {
                        if (Math.Abs(cplex.ObjValue - bestFind.solutionTravelTime) > p.doublePrecision)
                        {
                            Misc.errOut("Travel time in the cplex solution and recovered solution do not match!");
                        }
                    }
                    else if (p.minimizationType == MinimizationType.TotalTime)
                    {
                        if (Math.Abs(cplex.ObjValue - bestFind.solutionTotalTime) > p.doublePrecision)
                        {
                            Misc.errOut("Total time in the cplex solution and recovered solution do not match!");
                        }
                    }
                    else
                    {
                        Misc.errOut("Unknown secondary objective function!");
                    }

                    Console.WriteLine("Num vehicles:" + bestFind.vehicles.Count);
                    Console.WriteLine("Distance traveled:" + bestFind.solutionDist);
                    Console.WriteLine("Travel time:" + bestFind.solutionTravelTime);
                    Console.WriteLine("Energy consumed:" + bestFind.solutionDist);
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
         * This function check wheter the solution returned from cplex is really correct
         * (this is actually selfcheck to see if my model is correct)
         */
        public void checkCplexSolutionv2(double[] x, MILPData md, int numVehs)
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
                    if (x[varIndex] > 0)
                    {
                        numExit++;
                    }
                }
                foreach (int varIndex in varIndexXijkIn)
                {
                    if (x[varIndex] > 0)
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
                    if (numExit != 0 || numInto != 1)
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
                        //In some cases the cplex throws this error, and the optimizations stops
                        //When I rerun the solution for the instance where the error occured, no error appears??
                        //Perhaps it has something to do with multiple parallel runs (as I do it on for example toggrupa-01)
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
                            count++;
                            departureTime += x[pair.Value];
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
                    if (Math.Abs(departureTime - computedDepartureTime) > p.doublePrecision)
                    {
                        Misc.errOut("Departure times in cplex solution do not match!");
                    }
                }
            }

            if (numInputArcsEndDepots != numVehs)
            {
                Misc.errOut("Number of input arcs into ending depots differs from num of vehicles!");
            }
        }

        /*Returns instance of solution extracted from cplex solution array x=A^{-1}b
         */
        private Solution getSolutionFromCplexV2(double[] x)
        {
            //Create instance of solution
            Solution s = new Solution(p);
            int numVeh = 0;
            foreach (KeyValuePair<(UserMILP, UserMILP, int), int> pair in md.Xijk)
            {
                if (x[pair.Value] > 0 && pair.Key.Item1 == md.depot0)
                {
                    numVeh++;
                    //New vehicle
                    Vehicle v = new Vehicle(p, s.vehicles.Count, s);
                    //Arc 0->j
                    UserMILP uj = pair.Key.Item2;
                    while (true)
                    {
                        //If ending depot is found, close vehicle and update all the values
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
                            UserMILP ui = uj;

                            double beginTimeCplex = x[ui.serviceStartTimeVarInd];
                            if (!Misc.EqualDoubleValues(beginTimeCplex, uj.u.beginTime, p.doublePrecision))
                            {
                                Misc.errOut("Begin times do not coincide!");
                            }

                            //Console.WriteLine("Computed begin time:"+uj.u.beginTime);
                            //Console.WriteLine("Cplex begin time:" + beginTime);

                            double restBatCap = x[ui.restEnergyVarInd];
                            //Console.WriteLine("Computed rest bat cap:" + uj.u.restBatCapAtArrival);
                            //Console.WriteLine("Cplex rest bat cap:" + restBatCap);

                            if (!Misc.EqualDoubleValues(restBatCap, uj.u.restBatCapAtArrival, p.doublePrecision))
                            {
                                Misc.errOut("Rest bat capacity at arrival do not coincide!");
                            }

                            //Console.WriteLine("Computed departure time:" + uj.u.departureTime);

                            //Checking departure time for each user excpet ending depot which actually does not have a departure time
                            if (!md.endingDepots.Contains(ui))
                            {
                                int count = 0;
                                double departureTimeCplex = 0;
                                foreach (KeyValuePair<(UserMILP, UserMILP, int), int> pair3 in md.Tijk)
                                {
                                    if (x[pair3.Value] > p.doublePrecision && pair3.Key.Item1 == ui)
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
                                //Console.WriteLine("Cplex departure time:" + departureTime);

                                if (!Misc.EqualDoubleValues(departureTimeCplex, uj.u.departureTime, p.doublePrecision))
                                {
                                    Misc.errOut("Departure times do not coincide!");
                                }
                            }
                            foreach (KeyValuePair<(UserMILP, UserMILP, int), int> pair2 in md.Xijk)
                            {
                                if (pair2.Key.Item1 == ui && x[pair2.Value] > 0)
                                {
                                    //Arc i->j is used
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

