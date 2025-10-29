using CPLEX_TDTSPTW;
using ILOG.Concert;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CPLEX_TDLEVRPTW
{
    public static partial class PopulateBasicModel
    {
        public static BasicModel populateModelMatricesOld(MILPDataOld md, Params p)
        {
            //Init basic model vectors
            BasicModel bm = new BasicModel(md.numVars);
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
                // Spent a lot of time debugging this, because it was not zero, and it has to be zero in cases when xijk=0
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
                bm.ub[varIndexTWStart] = um == md.depot0 ? um.u.etw : um.u.ltw;

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

            /*Only 1 entry arc in each customer in all time buckets
             *              * */
            //tex: Equation $\forall j \in V$ $\sum_{k \in K} \sum_{i \in V_{0} \cup F', i \neq j} x_{ij}^k=1$ 
            foreach (UserMILP j in md.getV())
            {
                double[] row = new double[md.numVars];
                foreach (UserMILP i in md.getV0_())
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
             * At max 1 entry arc in each CS (some could have zero) in all time buckets
             */
            //tex: $\forall j \in F' \cup ED$ $\sum_{k \in K} \sum_{i \in V_{0} \cup F', i \neq j} x_{ij}^k \leq 1$ 
            foreach (UserMILP j in md.getFNm_())
            {
                double[] row = new double[md.numVars];
                foreach (UserMILP i in md.getV0_())
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
            //tex: $\forall j \in V \cup F'$ $\sum_{k \in K} \sum_{i \in V_{ED} \cup F', i \neq j} x_{ji}^k - \sum_{k \in K} \sum_{i \in V_{0} \cup F', i \neq j} x_{ij}^k = 0$ 
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

            /*Only 1 entry arc for each ending depot - this was removed as second equation was rewritten to include it
 *              * */
            //tex: Equation $\forall j \in MED$ $\sum_{k \in K} \sum_{i \in V \cup F', i \neq j} x_{ij}^k=1$ 
            //foreach (UserMILP j in md.getMED())
            //{
            //    double[] row = new double[md.numVars];
            //    foreach (UserMILP i in md.getV_())
            //    {
            //        for (int k = 0; k < md.kBuckets; k++)
            //        {
            //            if (md.Xijk.ContainsKey((i, j, k)))
            //            {
            //                row[md.Xijk[(i, j, k)]] = 1;
            //            }
            //        }
            //    }
            //    if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
            //    {
            //        Misc.errOut("All values in a row should not be zero!");
            //    }
            //    bm.A.Add(row);
            //    bm.b.Add(1);
            //    bm.eqType.Add("lowerOrEqual");
            //}

            // Equation for time feasibility for arcs leaving customers and depot
            //tex: $\forall i \in V_0, \forall j \in V_{ED} \cup F'~ i\neq j $ $\tau_i+s_i \sum_{k \in K}x_{ij}^k+\sum_{k \in K} (\Theta_{ij}^k \zeta_{ij}^k + \eta_{ij}^k x_{ij}^k) - (l_0+gQ)(1-\sum_{k \in K} x_{ij}^k) \leq \tau_j$ 

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
                            row[md.Xijk[(i, j, k)]] = p.getSectionFromLinTime(i.u, j.u, k) + p.depot.ltw + p.refuelRate * p.batCap + i.u.serviceTime;
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
                        bm.b.Add(p.depot.ltw + p.refuelRate * p.batCap);
                        bm.eqType.Add("lowerOrEqual");
                    }
                }
            }

            // Equation for time feasibility for arcs leaving CSs
            //tex: $\forall i \in F', \forall j \in V_{ED} \cup F'~ i\neq j$ $\tau_i+g(Q-y_i)+\sum_{k \in K} (\Theta_{ij}^k \zeta_{ij}^k + \eta_{ij}^k x_{ij}^k) - (l_0+gQ)(1-\sum_{k \in K}x_{ij}^k) \leq \tau_j$ 

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

            // Equation for the computation of departure time at customer based on service start time (probably can be ommited in future versions)
            //tex: $\forall i \in V_{0} $ $\sum_{k \in K} \sum_{j \in V_{ED} \cup F', i \neq j} \zeta_{ij}^k=\tau_i+s_i$ 

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

            // Equation for the computation of departure start time at CS based on service start time (probably can be ommited in future versions)
            //tex: $\forall i \in F' $ $\sum_{k \in K} \sum_{j \in V_{ED} \cup F', i \neq j} \zeta_{ij}^k \leq \tau_i+g(Q-y_i)+(l_0+gQ)(1-\sum_{k \in K} \sum_{j \in V_{ED} \cup F', i \neq j} x_{ij}^k)$ 

            foreach (UserMILP i in md.getF_())
            {
                double[] row = new double[md.numVars];
                for (int k = 0; k < md.kBuckets; k++)
                {
                    foreach (UserMILP j in md.getVNm_())
                    {
                        if (md.Tijk.ContainsKey((i, j, k)))
                        {
                            row[md.Tijk[(i, j, k)]] = 1;
                        }
                        if (md.Xijk.ContainsKey((i, j, k)))
                        {
                            row[md.Xijk[(i, j, k)]] = p.depot.ltw + p.refuelRate * p.batCap;
                        }
                    }
                }
                row[i.serviceStartTimeVarInd] = -1;
                row[i.restEnergyVarInd] = p.refuelRate;
                if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                {
                    Misc.errOut("All values in a row should not be zero!");
                }
                bm.A.Add(row);
                bm.b.Add(2 * p.refuelRate * p.batCap + p.depot.ltw);
                bm.eqType.Add("lowerOrEqual");
            }

            //////BEfore
            //foreach (UserMILP i in md.getF_())
            //{
            //    double[] row = new double[md.numVars];
            //    for (int k = 0; k < md.kBuckets; k++)
            //    {
            //        foreach (UserMILP j in md.getVNm_())
            //        {
            //            if (md.Tijk.ContainsKey((i, j, k)))
            //            {
            //                row[md.Tijk[(i, j, k)]] = -1;
            //            }
            //        }
            //    }
            //    row[i.serviceStartTimeVarInd] = 1;
            //    row[i.restEnergyVarInd] = -p.refuelRate;
            //    if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
            //    {
            //        Misc.errOut("All values in a row should not be zero!");
            //    }
            //    bm.A.Add(row);
            //    bm.b.Add(-p.refuelRate * p.batCap);
            //    bm.eqType.Add("equal");
            //}

            // Equation for the computation of departure start time at CS based on service start time (probably can be ommited in future versions)
            //tex: $\forall i \in F' $ $\sum_{k \in K} \sum_{j \in V_{ED} \cup F', i \neq j} \zeta_{ij}^k \geq \tau_i+g(Q-y_i)-(l_0+gQ)(1-\sum_{k \in K} \sum_{j \in V_{ED} \cup F', i \neq j} x_{ij}^k)$ 

            foreach (UserMILP i in md.getF_())
            {
                double[] row = new double[md.numVars];
                for (int k = 0; k < md.kBuckets; k++)
                {
                    foreach (UserMILP j in md.getVNm_())
                    {
                        if (md.Tijk.ContainsKey((i, j, k)))
                        {
                            row[md.Tijk[(i, j, k)]] = 1;
                        }
                        if (md.Xijk.ContainsKey((i, j, k)))
                        {
                            row[md.Xijk[(i, j, k)]] = -(p.depot.ltw + p.refuelRate * p.batCap);
                        }
                    }
                }
                row[i.serviceStartTimeVarInd] = -1;
                row[i.restEnergyVarInd] = p.refuelRate;
                if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                {
                    Misc.errOut("All values in a row should not be zero!");
                }
                bm.A.Add(row);
                bm.b.Add(-p.depot.ltw);
                bm.eqType.Add("greaterOrEqual");
            }



            // Equation for time feasibility for arcs leaving customers and depot with max function on begin time - tau j
            //tex: $\forall i \in V_0, \forall j \in V_{ED} \cup F'~ i\neq j $ $\tau_i+s_i \sum_{k \in K}x_{ij}^k+\sum_{k \in K} (\Theta_{ij}^k \zeta_{ij}^k + \eta_{ij}^k x_{ij}^k) + (l_0+gQ)(1-\sum_{k \in K} x_{ij}^k) +2(l_0+gQ)b_{ij} \geq \tau_j$ 

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
                            row[md.Xijk[(i, j, k)]] = p.getSectionFromLinTime(i.u, j.u, k) - (p.depot.ltw + p.refuelRate * p.batCap) + i.u.serviceTime;
                        }
                    }
                    if (any)
                    {
                        row[i.serviceStartTimeVarInd] = 1;
                        row[j.serviceStartTimeVarInd] = -1;
                        row[md.Bij[(i, j)]] = 2 * (p.depot.ltw + p.refuelRate * p.batCap);
                        if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                        {
                            Misc.errOut("All values in a row should not be zero!");
                        }
                        bm.A.Add(row);
                        bm.b.Add(-(p.depot.ltw + p.refuelRate * p.batCap));
                        bm.eqType.Add("greaterOrEqual");
                    }
                }
            }

            // Equation for time feasibility for arcs leaving customers and depot with max function on begin time - early time winow ej
            //tex: $\forall i \in V_0 \cup F', \forall j \in V_{ED} \cup F'~ i\neq j $ $\tau_j \leq e_j+2(l_0+gQ)-b_j(l_0+gQ) -\sum_{k \in K}x_{ij}^k(l_0+gQ)$ 

            foreach (UserMILP i in md.getV0_())
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
                            row[md.Xijk[(i, j, k)]] = p.depot.ltw + p.refuelRate * p.batCap;
                        }
                    }
                    if (any)
                    {
                        row[j.serviceStartTimeVarInd] = 1;
                        row[md.Bij[(i, j)]] = (p.depot.ltw + p.refuelRate * p.batCap);
                        if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                        {
                            Misc.errOut("All values in a row should not be zero!");
                        }
                        bm.A.Add(row);
                        bm.b.Add(j.u.etw + 2 * (p.depot.ltw + p.refuelRate * p.batCap));
                        bm.eqType.Add("lowerOrEqual");
                    }
                }
            }

            // Equation for time feasibility for arcs leaving CSs - max function begin time - tau_j
            //tex: $\forall i \in F', \forall j \in V_{ED} \cup F'~ i\neq j$ $\tau_i+g(Q-y_i)+\sum_{k \in K} (\Theta_{ij}^k \zeta_{ij}^k + \eta_{ij}^k x_{ij}^k) + (l_0+gQ)(1-\sum_{k \in K}x_{ij}^k)+2b_{ij}(l_0+gQ) \geq \tau_j$ 
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
                            row[md.Xijk[(i, j, k)]] = p.getSectionFromLinTime(i.u, j.u, k) - (p.depot.ltw + p.refuelRate * p.batCap);
                        }
                    }
                    if (any)
                    {
                        row[i.serviceStartTimeVarInd] = 1;
                        row[j.serviceStartTimeVarInd] = -1;
                        row[i.restEnergyVarInd] = -p.refuelRate;
                        row[md.Bij[(i, j)]] = 2 * (p.refuelRate * p.batCap + p.depot.ltw);
                        if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                        {
                            Misc.errOut("All values in a row should not be zero!");
                        }
                        bm.A.Add(row);
                        bm.b.Add(-(p.depot.ltw + 2 * p.refuelRate * p.batCap));
                        //bm.b.Add(p.depot.ltw);
                        bm.eqType.Add("greaterOrEqual");
                    }
                }
            }

            //// Equation for time feasibility for arcs leaving CSs - max function begin time - etw
            ////tex: $\forall i \in F', \forall j \in V_{ED} \cup F'~ i\neq j$ $\tau_j \leq e_j +2(1-b_{ij})(l_0+gQ)+b_{ij}(l_0+gQ)-\sum_{k \in K}x_{ij}^k(l_0+gQ)$ 
            //foreach (UserMILP i in md.getF_())
            //{
            //    foreach (UserMILP j in md.getVNm_())
            //    {
            //        double[] row = new double[md.numVars];
            //        bool any = false;
            //        for (int k = 0; k < md.kBuckets; k++)
            //        {
            //            if (md.Xijk.ContainsKey((i, j, k)))
            //            {
            //                any = true;
            //                row[md.Xijk[(i, j, k)]] = p.depot.ltw + p.refuelRate * p.batCap;
            //            }
            //        }
            //        if (any)
            //        {
            //            row[j.serviceStartTimeVarInd] = 1;
            //            row[md.Bij[(i, j)]] = p.depot.ltw + p.refuelRate * p.batCap;
            //            if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
            //            {
            //                Misc.errOut("All values in a row should not be zero!");
            //            }
            //            bm.A.Add(row);
            //            bm.b.Add(j.u.etw + 2 * (p.depot.ltw + p.refuelRate * p.batCap));
            //            bm.eqType.Add("lowerOrEqual");
            //        }
            //    }
            //}

            // Equation for ensuring that each departure time of customer/CS is within its appropriate breakpointes
            // determined by breakpoints of time buckets and latest departure times at customers/CSs
            // For customers the search space is adequatly reduced, while for CSs this is not the case
            //tex: $\forall i \in V_0, \forall j \in V_{ED} \cup F'~ i\neq j, \forall k \in K$ $x_{ij}^k \max{ (w_k,e_i+s_i)} \leq \zeta_{ij}^k \leq \min{ (w_{k+1},l_i+s_i)}x_{ij}^k$ 
            // $\forall i \in F', \forall j \in V_{ED} \cup F'~ i\neq j, \forall k \in K$ $x_{ij}^k \max{ (w_k,e_i+s_i)} \leq \zeta_{ij}^k \leq \min{ (w_{k+1},l_i+gQ)}x_{ij}^k$
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
            //tex: $\forall i \in V_0 \cup F', \forall j \in V_{ED} \cup F'~ i\neq j, u_j \leq u_i- \sum_{k \in K}x_{ij}^k (q_i+C)+C$ 

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
            //tex: $\forall i \in V, \forall j \in V_{ED} \cup F'~ i\neq j, y_j \leq y_i- \sum_{k \in K}x_{ij}^k (e_{ij}^k+Q)+Q$ 
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

            // Equation for reamining battery capcity flow conservation (energy) for customers
            //tex: $\forall i \in F'_0, \forall j \in V_{ED} \cup F'~ i\neq j, y_j \leq Q- \sum_{k \in K}x_{ij}^k e_{ij}^k$ 

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
            //Exact yj is necesearry only in TD cases when we jave to know the departure times
            //tex: $\forall i \in V, \forall j \in V_{ED} \cup F'~ i\neq j, y_j \geq y_i- \sum_{k \in K}x_{ij}^k (e_{ij}^k-Q)-Q$ 
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


            // Equation for reamining battery capcity flow conservation (energy) for customers - exact yj
            //tex: $\forall i \in F'_0, \forall j \in V_{ED} \cup F'~ i\neq j, y_j \geq \sum_{k \in K}x_{ij}^k (Q-e_{ij}^k)$ 

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
                            row[md.Xijk[(i, j, k)]] = -p.batCap + p.ener(i.u, j.u, k);
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
    }
}


//using CPLEX_TDTSPTW;
//using ILOG.Concert;
//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Text;
//using System.Threading.Tasks;

//namespace CPLEX_TDLEVRPTW
//{
//    public static partial class PopulateBasicModel
//    {
//        public static BasicModel populateModelMatricesOld(MILPDataOld md, Params p)
//        {
//            //Init basic model vectors
//            BasicModel bm = new BasicModel(md.numVars);
//            //Upper and lower bounds

//            //XIJK - binary decision variable (0,1) integer
//            foreach (KeyValuePair<(UserMILP, UserMILP, int), int> element in md.Xijk)
//            {
//                int varIndexXijk = element.Value;
//                bm.xtb[varIndexXijk] = NumVarType.Int;
//                bm.lb[varIndexXijk] = 0;
//                bm.ub[varIndexXijk] = 1;
//            }
//            //TIJK - float variable actually representign departure time
//            foreach (KeyValuePair<(UserMILP, UserMILP, int), int> element in md.Tijk)
//            {
//                int varIndexTijk = element.Value;
//                bm.xtb[varIndexTijk] = NumVarType.Float;
//                // Spent a lot of time debugging this, because it was not zero, and it has to be zero in cases when xijk=0
//                bm.lb[varIndexTijk] = 0;
//                UserMILP u = element.Key.Item1;
//                if (md.getV0().Contains(u))
//                {
//                    bm.ub[varIndexTijk] = u.u.ltw + u.u.serviceTime;
//                }
//                else if (md.getF_().Contains(u))
//                {
//                    bm.ub[varIndexTijk] = u.u.ltw + p.refuelRate * p.batCap;
//                }
//                else
//                {
//                    Misc.errOut("Error! User is not ni F' or V0!");
//                }
//            }

//            //BIJ - binary decision variable (0,1) integer
//            foreach (KeyValuePair<(UserMILP, UserMILP), int> element in md.Bij)
//            {
//                int varIndexBij = element.Value;
//                bm.xtb[varIndexBij] = NumVarType.Int;
//                bm.lb[varIndexBij] = 0;
//                bm.ub[varIndexBij] = 1;
//            }

//            //Users
//            foreach (UserMILP um in md.getV0Nm_())
//            {
//                //Service start time windows bound
//                int varIndexTWStart = um.serviceStartTimeVarInd;
//                bm.xtb[varIndexTWStart] = NumVarType.Float;
//                bm.lb[varIndexTWStart] = um.u.etw;
//                bm.ub[varIndexTWStart] = um == md.depot0 ? um.u.etw : um.u.ltw;

//                //Load capacity bound
//                int varIndexLoad = um.restLoadVarInd;
//                bm.xtb[varIndexLoad] = NumVarType.Float;
//                bm.lb[varIndexLoad] = um == md.depot0 ? p.loadCap : 0;
//                bm.ub[varIndexLoad] = p.loadCap;

//                //Energy capacity bound
//                int varIndexEnergy = um.restEnergyVarInd;
//                bm.xtb[varIndexEnergy] = NumVarType.Float;
//                bm.lb[varIndexEnergy] = (um == md.depot0) ? p.batCap : 0;
//                bm.ub[varIndexEnergy] = p.batCap;
//            }

//            //Equations Ax=b

//            /*Only 1 entry arc in each customer in all time buckets
//                        *              * */
//            //tex: Equation $\forall j \in V$ $\sum_{k \in K} \sum_{i \in V_{0} \cup F', i \neq j} x_{ij}^k=1$ 
//            foreach (UserMILP j in md.getV())
//            {
//                double[] row = new double[md.numVars];
//                foreach (UserMILP i in md.getV0_())
//                {
//                    for (int k = 0; k < md.kBuckets; k++)
//                    {
//                        if (md.Xijk.ContainsKey((i, j, k)))
//                        {
//                            row[md.Xijk[(i, j, k)]] = 1;
//                        }
//                    }
//                }
//                if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
//                {
//                    Misc.errOut("All values in a row should not be zero!");
//                }
//                bm.A.Add(row);
//                bm.b.Add(1);
//                bm.eqType.Add("equal");
//            }

//            /*
//                        * At max 1 entry arc in each CS (some could have zero) in all time buckets
//                        */
//            //tex: $\forall j \in F' \cup ED$ $\sum_{k \in K} \sum_{i \in V_{0} \cup F', i \neq j} x_{ij}^k \leq 1$ 
//            foreach (UserMILP j in md.getFNm_())
//            {
//                double[] row = new double[md.numVars];
//                foreach (UserMILP i in md.getV0_())
//                {
//                    for (int k = 0; k < md.kBuckets; k++)
//                    {
//                        if (md.Xijk.ContainsKey((i, j, k)))
//                        {
//                            row[md.Xijk[(i, j, k)]] = 1;
//                        }

//                    }
//                }
//                if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
//                {
//                    Misc.errOut("All values in a row should not be zero!");
//                }
//                bm.A.Add(row);
//                bm.b.Add(1);
//                bm.eqType.Add("lowerOrEqual");
//            }


//            // Flow conservation-> number of exit arcs has to be equal to the number of entry arcs for all arcs in all time periods
//            //tex: $\forall j \in V \cup F'$ $\sum_{k \in K} \sum_{i \in V_{ED} \cup F', i \neq j} x_{ji}^k - \sum_{k \in K} \sum_{i \in V_{0} \cup F', i \neq j} x_{ij}^k = 0$ 
//            foreach (UserMILP j in md.getV_())
//            {
//                double[] row = new double[md.numVars];
//                for (int k = 0; k < md.kBuckets; k++)
//                {
//                    foreach (UserMILP i in md.getVNm_())
//                    {
//                        if (md.Xijk.ContainsKey((j, i, k)))
//                        {
//                            row[md.Xijk[(j, i, k)]] += 1;
//                        }
//                    }
//                    foreach (UserMILP i in md.getV0_())
//                    {
//                        if (md.Xijk.ContainsKey((i, j, k)))
//                        {
//                            row[md.Xijk[(i, j, k)]] -= 1;
//                        }
//                    }
//                }
//                if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
//                {
//                    Misc.errOut("All values in a row should not be zero!");
//                }
//                bm.A.Add(row);
//                bm.b.Add(0);
//                bm.eqType.Add("equal");
//            }

//            /*Only 1 entry arc for each ending depot - this was removed as second equation was rewritten to include it
//            *              * */
//            //tex: Equation $\forall j \in MED$ $\sum_{k \in K} \sum_{i \in V \cup F', i \neq j} x_{ij}^k=1$ 
//            //foreach (UserMILP j in md.getMED())
//            //{
//            //    double[] row = new double[md.numVars];
//            //    foreach (UserMILP i in md.getV_())
//            //    {
//            //        for (int k = 0; k < md.kBuckets; k++)
//            //        {
//            //            if (md.Xijk.ContainsKey((i, j, k)))
//            //            {
//            //                row[md.Xijk[(i, j, k)]] = 1;
//            //            }
//            //        }
//            //    }
//            //    if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
//            //    {
//            //        Misc.errOut("All values in a row should not be zero!");
//            //    }
//            //    bm.A.Add(row);
//            //    bm.b.Add(1);
//            //    bm.eqType.Add("lowerOrEqual");
//            //}

//            // Equation for time feasibility for arcs leaving customers and depot
//            //tex: $\forall i \in V_0, \forall j \in V_{ED} \cup F'~ i\neq j $ $\tau_i+s_i \sum_{k \in K}x_{ij}^k+\sum_{k \in K} (\Theta_{ij}^k \zeta_{ij}^k + \eta_{ij}^k x_{ij}^k) - (l_0+gQ)(1-\sum_{k \in K} x_{ij}^k) \leq \tau_j$ 

//            foreach (UserMILP i in md.getV0())
//            {
//                foreach (UserMILP j in md.getVNm_())
//                {
//                    double[] row = new double[md.numVars];
//                    bool any = false;
//                    for (int k = 0; k < md.kBuckets; k++)
//                    {
//                        if (md.Tijk.ContainsKey((i, j, k)) && md.Xijk.ContainsKey((i, j, k)))
//                        {
//                            any = true;

//                            row[md.Tijk[(i, j, k)]] = p.getSlopeFromLinTime(i.u, j.u, k);
//                            row[md.Xijk[(i, j, k)]] = p.getSectionFromLinTime(i.u, j.u, k) + p.depot.ltw + p.refuelRate * p.batCap + i.u.serviceTime;
//                        }
//                    }
//                    if (any)
//                    {
//                        row[i.serviceStartTimeVarInd] = 1;
//                        row[j.serviceStartTimeVarInd] = -1;
//                        if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
//                        {
//                            Misc.errOut("All values in a row should not be zero!");
//                        }
//                        bm.A.Add(row);
//                        bm.b.Add(p.depot.ltw + p.refuelRate * p.batCap);
//                        bm.eqType.Add("lowerOrEqual");
//                    }
//                }
//            }

//            // Equation for time feasibility for arcs leaving CSs
//            //tex: $\forall i \in F', \forall j \in V_{ED} \cup F'~ i\neq j$ $\tau_i+g(Q-y_i)+\sum_{k \in K} (\Theta_{ij}^k \zeta_{ij}^k + \eta_{ij}^k x_{ij}^k) - (l_0+gQ)(1-\sum_{k \in K}x_{ij}^k) \leq \tau_j$ 

//            foreach (UserMILP i in md.getF_())
//            {
//                foreach (UserMILP j in md.getVNm_())
//                {
//                    double[] row = new double[md.numVars];
//                    bool any = false;
//                    for (int k = 0; k < md.kBuckets; k++)
//                    {
//                        if (md.Tijk.ContainsKey((i, j, k)) && md.Xijk.ContainsKey((i, j, k)))
//                        {
//                            any = true;
//                            row[md.Tijk[(i, j, k)]] = p.getSlopeFromLinTime(i.u, j.u, k);
//                            row[md.Xijk[(i, j, k)]] = p.getSectionFromLinTime(i.u, j.u, k) + p.depot.ltw + p.refuelRate * p.batCap;
//                        }
//                    }
//                    if (any)
//                    {
//                        row[i.serviceStartTimeVarInd] = 1;
//                        row[j.serviceStartTimeVarInd] = -1;
//                        row[i.restEnergyVarInd] = -p.refuelRate;
//                        if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
//                        {
//                            Misc.errOut("All values in a row should not be zero!");
//                        }
//                        bm.A.Add(row);
//                        bm.b.Add(p.depot.ltw);
//                        bm.eqType.Add("lowerOrEqual");
//                    }
//                }
//            }

//            // Equation for the computation of service start time at customer (probably can be ommited in future versions)
//            //tex: $\forall i \in V_{0} $ $\sum_{k \in K} \sum_{j \in V_{ED} \cup F', i \neq j} \zeta_{ij}^k=\tau_i+s_i$ 

//            foreach (UserMILP i in md.getV0())
//            {
//                double[] row = new double[md.numVars];
//                for (int k = 0; k < md.kBuckets; k++)
//                {
//                    foreach (UserMILP j in md.getVNm_())
//                    {
//                        if (md.Tijk.ContainsKey((i, j, k)))
//                        {
//                            row[md.Tijk[(i, j, k)]] = -1;
//                        }
//                    }
//                }
//                row[i.serviceStartTimeVarInd] = 1;
//                if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
//                {
//                    Misc.errOut("All values in a row should not be zero!");
//                }
//                bm.A.Add(row);
//                bm.b.Add(-i.u.serviceTime);
//                bm.eqType.Add("equal");
//            }

//            // Equation for the computation of service start time at CS (probably can be ommited in future versions)
//            //tex: $\forall i \in F' $ $\sum_{k \in K} \sum_{j \in V_{ED} \cup F', i \neq j} \zeta_{ij}^k=\tau_i+g(Q-y_i)$ 
//            foreach (UserMILP i in md.getF_())
//            {
//                double[] row = new double[md.numVars];
//                for (int k = 0; k < md.kBuckets; k++)
//                {
//                    foreach (UserMILP j in md.getVNm_())
//                    {
//                        if (md.Tijk.ContainsKey((i, j, k)))
//                        {
//                            row[md.Tijk[(i, j, k)]] = -1;
//                        }
//                    }
//                }
//                row[i.serviceStartTimeVarInd] = 1;
//                row[i.restEnergyVarInd] = -p.refuelRate;
//                if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
//                {
//                    Misc.errOut("All values in a row should not be zero!");
//                }
//                bm.A.Add(row);
//                bm.b.Add(-p.refuelRate * p.batCap);
//                bm.eqType.Add("equal");
//            }

//            // Equation for time feasibility for arcs leaving customers and depot with max function on begin time - tau j
//            //tex: $\forall i \in V_0, \forall j \in V_{ED} \cup F'~ i\neq j $ $\tau_i+s_i \sum_{k \in K}x_{ij}^k+\sum_{k \in K} (\Theta_{ij}^k \zeta_{ij}^k + \eta_{ij}^k x_{ij}^k) - (l_0+gQ)(1-\sum_{k \in K} x_{ij}^k) +2(l_0+gQ)b_{ij} \geq \tau_j$ 
//            //tex: $\forall i \in V_0, \forall j \in V_{ED} \cup F'~ i\neq j $ $\tau_i+s_i \sum_{k \in K}x_{ij}^k+\sum_{k \in K} (\Theta_{ij}^k \zeta_{ij}^k + \eta_{ij}^k x_{ij}^k) + (l_0+gQ)(1-\sum_{k \in K} x_{ij}^k) +2(l_0+gQ)b_{ij} \geq \tau_j$ 

//            foreach (UserMILP i in md.getV0())
//            {
//                foreach (UserMILP j in md.getVNm_())
//                {
//                    double[] row = new double[md.numVars];
//                    bool any = false;
//                    for (int k = 0; k < md.kBuckets; k++)
//                    {
//                        if (md.Tijk.ContainsKey((i, j, k)) && md.Xijk.ContainsKey((i, j, k)))
//                        {
//                            any = true;

//                            row[md.Tijk[(i, j, k)]] = p.getSlopeFromLinTime(i.u, j.u, k);
//                            //Ovaj mora biti tako da bi radilo
//                            //row[md.Xijk[(i, j, k)]] = p.getSectionFromLinTime(i.u, j.u, k) + p.depot.ltw + p.refuelRate * p.batCap + i.u.serviceTime;
//                            row[md.Xijk[(i, j, k)]] = p.getSectionFromLinTime(i.u, j.u, k) - (p.depot.ltw + p.refuelRate * p.batCap) + i.u.serviceTime;
//                        }
//                    }
//                    if (any)
//                    {
//                        row[i.serviceStartTimeVarInd] = 1;
//                        row[j.serviceStartTimeVarInd] = -1;
//                        row[md.Bij[(i, j)]] = 2 * (p.depot.ltw + p.refuelRate * p.batCap);
//                        if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
//                        {
//                            Misc.errOut("All values in a row should not be zero!");
//                        }
//                        bm.A.Add(row);
//                        bm.b.Add(-(p.depot.ltw + p.refuelRate * p.batCap));
//                        bm.eqType.Add("greaterOrEqual");
//                    }
//                }
//            }

//            // Equation for time feasibility for arcs leaving customers and depot with max function on begin time - early time winow ej
//            //tex: $\forall i \in V_0, \forall j \in V_{ED} \cup F'~ i\neq j $ $\tau_j \leq e_j+(1-b_j)2(l_0+gQ)+b_j(l_0+gQ) -\sum_{k \in K}x_{ij}^k(l_0+gQ)$ 

//            foreach (UserMILP i in md.getV0())
//            {
//                foreach (UserMILP j in md.getVNm_())
//                {
//                    double[] row = new double[md.numVars];
//                    bool any = false;
//                    for (int k = 0; k < md.kBuckets; k++)
//                    {
//                        if (md.Xijk.ContainsKey((i, j, k)))
//                        {
//                            any = true;
//                            row[md.Xijk[(i, j, k)]] = p.depot.ltw + p.refuelRate * p.batCap;
//                        }
//                    }
//                    if (any)
//                    {
//                        row[j.serviceStartTimeVarInd] = 1;
//                        row[md.Bij[(i, j)]] = (p.depot.ltw + p.refuelRate * p.batCap);
//                        if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
//                        {
//                            Misc.errOut("All values in a row should not be zero!");
//                        }
//                        bm.A.Add(row);
//                        bm.b.Add(j.u.etw + 2 * (p.depot.ltw + p.refuelRate * p.batCap));
//                        bm.eqType.Add("lowerOrEqual");
//                    }
//                }
//            }

//            // Equation for time feasibility for arcs leaving CSs - max function begin time - tau_j
//            //tex: $\forall i \in F', \forall j \in V_{ED} \cup F'~ i\neq j$ $\tau_i+g(Q-y_i)+\sum_{k \in K} (\Theta_{ij}^k \zeta_{ij}^k + \eta_{ij}^k x_{ij}^k) - (l_0+gQ)(1-\sum_{k \in K}x_{ij}^k)+2b_{ij}(l_0+gQ) \geq \tau_j$ 
//            //tex: $\forall i \in F', \forall j \in V_{ED} \cup F'~ i\neq j$ $\tau_i+g(Q-y_i)+\sum_{k \in K} (\Theta_{ij}^k \zeta_{ij}^k + \eta_{ij}^k x_{ij}^k) + (l_0+gQ)(1-\sum_{k \in K}x_{ij}^k)+2b_{ij}(l_0+gQ) \geq \tau_j$ 
//            foreach (UserMILP i in md.getF_())
//            {
//                foreach (UserMILP j in md.getVNm_())
//                {
//                    double[] row = new double[md.numVars];
//                    bool any = false;
//                    for (int k = 0; k < md.kBuckets; k++)
//                    {
//                        if (md.Tijk.ContainsKey((i, j, k)) && md.Xijk.ContainsKey((i, j, k)))
//                        {
//                            any = true;
//                            row[md.Tijk[(i, j, k)]] = p.getSlopeFromLinTime(i.u, j.u, k);
//                            //row[md.Xijk[(i, j, k)]] = p.getSectionFromLinTime(i.u, j.u, k) + p.depot.ltw + p.refuelRate * p.batCap;
//                            row[md.Xijk[(i, j, k)]] = p.getSectionFromLinTime(i.u, j.u, k) - (p.depot.ltw + p.refuelRate * p.batCap);
//                        }
//                    }
//                    if (any)
//                    {
//                        row[i.serviceStartTimeVarInd] = 1;
//                        row[j.serviceStartTimeVarInd] = -1;
//                        row[i.restEnergyVarInd] = -p.refuelRate;
//                        row[md.Bij[(i, j)]] = 2 * (p.refuelRate * p.batCap + p.depot.ltw);
//                        if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
//                        {
//                            Misc.errOut("All values in a row should not be zero!");
//                        }
//                        bm.A.Add(row);
//                        //bm.b.Add(p.depot.ltw);
//                        bm.b.Add(-(p.depot.ltw + 2 * p.refuelRate * p.batCap));
//                        bm.eqType.Add("greaterOrEqual");
//                    }
//                }
//            }

//            // Equation for time feasibility for arcs leaving CSs - max function begin time - etw
//            //tex: $\forall i \in F', \forall j \in V_{ED} \cup F'~ i\neq j$ $\tau_j \leq e_j +2(1-b_{ij})(l_0+gQ)+b_{ij}(l_0+gQ)-\sum_{k \in K}x_{ij}^k(l_0+gQ)$ 
//            foreach (UserMILP i in md.getF_())
//            {
//                foreach (UserMILP j in md.getVNm_())
//                {
//                    double[] row = new double[md.numVars];
//                    bool any = false;
//                    for (int k = 0; k < md.kBuckets; k++)
//                    {
//                        if (md.Xijk.ContainsKey((i, j, k)))
//                        {
//                            any = true;
//                            row[md.Xijk[(i, j, k)]] = p.depot.ltw + p.refuelRate * p.batCap;
//                        }
//                    }
//                    if (any)
//                    {
//                        row[j.serviceStartTimeVarInd] = 1;
//                        row[md.Bij[(i, j)]] = p.depot.ltw + p.refuelRate * p.batCap;
//                        if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
//                        {
//                            Misc.errOut("All values in a row should not be zero!");
//                        }
//                        bm.A.Add(row);
//                        bm.b.Add(j.u.etw + 2 * (p.depot.ltw + p.refuelRate * p.batCap));
//                        bm.eqType.Add("lowerOrEqual");
//                    }
//                }
//            }

//            // Equation for ensuring that each departure time of customer/CS is within its appropriate breakpointes
//            // determined by breakpoints of time buckets and latest departure times at customers/CSs
//            // For customers the search space is adequatly reduced, while for CSs this is not the case
//            //tex: $\forall i \in V_0, \forall j \in V_{ED} \cup F'~ i\neq j, \forall k \in K$ $x_{ij}^k \max{ (w_k,e_i+s_i)} \leq \zeta_{ij}^k \leq \min{ (w_{k+1},l_i+s_i)}x_{ij}^k$ 
//            // $\forall i \in F', \forall j \in V_{ED} \cup F'~ i\neq j, \forall k \in K$ $x_{ij}^k \max{ (w_k,e_i+s_i)} \leq \zeta_{ij}^k \leq \min{ (w_{k+1},l_i+gQ)}x_{ij}^k$
//            foreach (UserMILP i in md.getV0_())
//            {
//                foreach (UserMILP j in md.getVNm_())
//                {
//                    for (int k = 0; k < md.kBuckets; k++)
//                    {
//                        if (md.Tijk.ContainsKey((i, j, k)) && md.Xijk.ContainsKey((i, j, k)))
//                        {
//                            double[] row1 = new double[md.numVars];
//                            Tuple<double, double> boundaries = p.getBoundaries(k);
//                            row1[md.Xijk[(i, j, k)]] = Math.Max(boundaries.Item1, i.u.etw + i.u.serviceTime);
//                            row1[md.Tijk[(i, j, k)]] = -1;
//                            if (!row1.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
//                            {
//                                Misc.errOut("All values in a row should not be zero!");
//                            }
//                            bm.A.Add(row1);
//                            bm.b.Add(0);
//                            bm.eqType.Add("lowerOrEqual");

//                            double[] row2 = new double[md.numVars];
//                            if (md.getV0().Contains(i))
//                            {
//                                row2[md.Xijk[(i, j, k)]] = -Math.Min(boundaries.Item2, i.u.ltw + i.u.serviceTime);
//                            }
//                            else if (md.getF_().Contains(i))
//                            {
//                                row2[md.Xijk[(i, j, k)]] = -Math.Min(boundaries.Item2, i.u.ltw + p.refuelRate * p.batCap);
//                            }
//                            else
//                            {
//                                Misc.errOut("Not in either of sets F_ or V0!");
//                            }
//                            row2[md.Tijk[(i, j, k)]] = 1;
//                            if (!row2.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
//                            {
//                                Misc.errOut("All values in a row should not be zero!");
//                            }
//                            bm.A.Add(row2);
//                            bm.b.Add(0);
//                            bm.eqType.Add("lowerOrEqual");
//                        }
//                    }
//                }
//            }

//            // Equation for reamining load capcity flow conservation
//            //tex: $\forall i \in V_0 \cup F', \forall j \in V_{ED} \cup F'~ i\neq j, u_j \leq u_i- \sum_{k \in K}x_{ij}^k (q_i+C)+C$ 

//            foreach (UserMILP i in md.getV0_())
//            {
//                foreach (UserMILP j in md.getVNm_())
//                {
//                    double[] row = new double[md.numVars];
//                    row[j.restLoadVarInd] = 1;
//                    row[i.restLoadVarInd] = -1;
//                    bool any = false;
//                    for (int k = 0; k < md.kBuckets; k++)
//                    {
//                        if (md.Xijk.ContainsKey((i, j, k)))
//                        {
//                            row[md.Xijk[(i, j, k)]] = i.u.demand + p.loadCap;
//                            any = true;
//                        }
//                    }
//                    if (any)
//                    {
//                        if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
//                        {
//                            Misc.errOut("All values in a row should not be zero!");
//                        }
//                        bm.A.Add(row);
//                        bm.b.Add(p.loadCap);
//                        bm.eqType.Add("lowerOrEqual");
//                    }
//                }
//            }

//            // Equation for reamining battery capcity flow conservation (energy) for customers
//            //tex: $\forall i \in V, \forall j \in V_{ED} \cup F'~ i\neq j, y_j \leq y_i- \sum_{k \in K}x_{ij}^k (e_{ij}^k+Q)+Q$ 
//            foreach (UserMILP j in md.getVNm_())
//            {
//                foreach (UserMILP i in md.getV())
//                {
//                    double[] row = new double[md.numVars];
//                    row[j.restEnergyVarInd] = 1;
//                    row[i.restEnergyVarInd] = -1;
//                    bool any = false;
//                    for (int k = 0; k < md.kBuckets; k++)
//                    {
//                        if (md.Xijk.ContainsKey((i, j, k)))
//                        {
//                            row[md.Xijk[(i, j, k)]] = p.ener(i.u, j.u, k) + p.batCap;
//                            any = true;
//                        }
//                    }
//                    if (any)
//                    {
//                        if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
//                        {
//                            Misc.errOut("All values in a row should not be zero!");
//                        }
//                        bm.A.Add(row);
//                        bm.b.Add(p.batCap);
//                        bm.eqType.Add("lowerOrEqual");
//                    }
//                }
//            }

//            // Equation for reamining battery capcity flow conservation (energy) for customers
//            //tex: $\forall i \in F'_0, \forall j \in V_{ED} \cup F'~ i\neq j, y_j \leq Q- \sum_{k \in K}x_{ij}^k e_{ij}^k$ 

//            foreach (UserMILP j in md.getVNm_())
//            {

//                foreach (UserMILP i in md.getF0_())
//                {
//                    double[] row = new double[md.numVars];
//                    row[j.restEnergyVarInd] = 1;
//                    bool any = false;
//                    for (int k = 0; k < md.kBuckets; k++)
//                    {
//                        if (md.Xijk.ContainsKey((i, j, k)))
//                        {
//                            row[md.Xijk[(i, j, k)]] = p.ener(i.u, j.u, k);
//                            any = true;
//                        }
//                    }
//                    if (any)
//                    {
//                        if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
//                        {
//                            Misc.errOut("All values in a row should not be zero!");
//                        }
//                        bm.A.Add(row);
//                        bm.b.Add(p.batCap);
//                        bm.eqType.Add("lowerOrEqual");
//                    }
//                }
//            }

//            // Equation for reamining battery capcity flow conservation (energy) for customers - exact yj
//            //Exact yj is necesearry only in TD cases when we jave to know the departure times
//            //tex: $\forall i \in V, \forall j \in V_{ED} \cup F'~ i\neq j, y_j \geq y_i- \sum_{k \in K}x_{ij}^k (e_{ij}^k-Q)-Q$ 
//            foreach (UserMILP j in md.getVNm_())
//            {
//                foreach (UserMILP i in md.getV())
//                {
//                    double[] row = new double[md.numVars];
//                    row[j.restEnergyVarInd] = 1;
//                    row[i.restEnergyVarInd] = -1;
//                    bool any = false;
//                    for (int k = 0; k < md.kBuckets; k++)
//                    {
//                        if (md.Xijk.ContainsKey((i, j, k)))
//                        {
//                            row[md.Xijk[(i, j, k)]] = p.ener(i.u, j.u, k) - p.batCap;
//                            any = true;
//                        }
//                    }
//                    if (any)
//                    {
//                        if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
//                        {
//                            Misc.errOut("All values in a row should not be zero!");
//                        }
//                        bm.A.Add(row);
//                        bm.b.Add(-p.batCap);
//                        bm.eqType.Add("greaterOrEqual");
//                    }
//                }
//            }


//            // Equation for reamining battery capcity flow conservation (energy) for customers - exact yj
//            //tex: $\forall i \in F'_0, \forall j \in V_{ED} \cup F'~ i\neq j, y_j \geq \sum_{k \in K}x_{ij}^k (Q-e_{ij}^k)$ 

//            foreach (UserMILP j in md.getVNm_())
//            {

//                foreach (UserMILP i in md.getF0_())
//                {
//                    double[] row = new double[md.numVars];
//                    row[j.restEnergyVarInd] = 1;
//                    bool any = false;
//                    for (int k = 0; k < md.kBuckets; k++)
//                    {
//                        if (md.Xijk.ContainsKey((i, j, k)))
//                        {
//                            row[md.Xijk[(i, j, k)]] = -p.batCap + p.ener(i.u, j.u, k);
//                            any = true;
//                        }
//                    }
//                    if (any)
//                    {
//                        if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
//                        {
//                            Misc.errOut("All values in a row should not be zero!");
//                        }
//                        bm.A.Add(row);
//                        bm.b.Add(0);
//                        bm.eqType.Add("greaterOrEqual");
//                    }
//                }
//            }
//            return bm;
//        }
//    }
//}
