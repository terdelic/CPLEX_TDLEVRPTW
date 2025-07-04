using CPLEX_TDLEVRPTW.CPLEX_TDTSPTW;
using CPLEX_TDTSPTW;
using ILOG.Concert;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection.Emit;
using System.Text;
using System.Threading.Tasks;

namespace CPLEX_TDLEVRPTW
{
    public static partial class PopulateBasicModel
    {
        public static BasicModel populateModelMatricesNew(MILPDataNew md, Params p)
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

            //ZISJK - binary decision variable (0,1) integer
            foreach (KeyValuePair<(UserMILP, UserMILP, UserMILP, int), int> element in md.Zisjk)
            {
                int varIndexXijk = element.Value;
                bm.xtb[varIndexXijk] = NumVarType.Int;
                bm.lb[varIndexXijk] = 0;
                bm.ub[varIndexXijk] = 1;
            }

            //Z_ISJK - binary decision variable (0,1) integer
            foreach (KeyValuePair<(UserMILP, UserMILP, UserMILP, int), int> element in md.Z_isjk)
            {
                int varIndexXijk = element.Value;
                bm.xtb[varIndexXijk] = NumVarType.Int;
                bm.lb[varIndexXijk] = 0;
                bm.ub[varIndexXijk] = 1;
            }


            //ZetaIJK - float variable actually representign departure time
            foreach (KeyValuePair<(UserMILP, UserMILP, int), int> element in md.Zetaijk)
            {
                int varIndexTijk = element.Value;
                bm.xtb[varIndexTijk] = NumVarType.Float;
                // Spent a lot of time debugging this, because it was not zero, and it has to be zero in cases when xijk=0
                bm.lb[varIndexTijk] = 0;
                UserMILP u = element.Key.Item1;
                bm.ub[varIndexTijk] = u.u.ltw + u.u.serviceTime;   
            }

            //ZetaISJK - float variable actually representign departure time at CS s
            foreach (KeyValuePair<(UserMILP, UserMILP, UserMILP, int), int> element in md.Zetaisjk)
            {
                int varIndexTijk = element.Value;
                bm.xtb[varIndexTijk] = NumVarType.Float;
                // Spent a lot of time debugging this, because it was not zero, and it has to be zero in cases when xijk=0
                bm.lb[varIndexTijk] = 0;
                UserMILP u = element.Key.Item2;
                bm.ub[varIndexTijk] = u.u.ltw + p.refuelRate * p.batCap;
            }

            //DeltaISJ - float variable representing additional time spent visting CS (travel+chargign)
            foreach (KeyValuePair<(UserMILP, UserMILP, UserMILP), int> element in md.Deltaisj)
            {
                int varIndexItem = element.Value;
                bm.xtb[varIndexItem] = NumVarType.Float;
                bm.lb[varIndexItem] = 0;
                UserMILP uI = element.Key.Item1;
                UserMILP uJ = element.Key.Item3;
                bm.ub[varIndexItem] =Math.Max(0, uJ.u.ltw - uI.u.etw);
            }


            //YISJ - float variable representing rest battery capacity at arrival at CS s
            foreach (KeyValuePair<(UserMILP, UserMILP, UserMILP), int> element in md.Yisj)
            {
                int varIndexItem = element.Value;
                bm.xtb[varIndexItem] = NumVarType.Float;
                bm.lb[varIndexItem] = 0;
                bm.ub[varIndexItem] = p.batCap;
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
            foreach (UserMILP um in md.getV0_cup_ED())
            {
                if(um == md.depot0)
                {
                    Console.WriteLine();
                }
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
            //tex: Equation $\sum_{k \in K} \sum_{i \in V_{0}, i \neq j} x_{ij}^k=1,~\forall j \in V$ 
            foreach (UserMILP j in md.getV())
            {
                double[] row = new double[md.numVars];
                foreach (UserMILP i in md.getV0())
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

            // Flow conservation-> number of exit arcs has to be equal to the number of entry arcs for all arcs in all time periods
            //tex: $\sum_{k \in K} \sum_{i \in V_{ED}, i \neq j} x_{ji}^k - \sum_{k \in K} \sum_{i \in V_{0}, i \neq j} x_{ij}^k = 0, ~\forall j \in V$ 
            foreach (UserMILP j in md.getV())
            {
                double[] row = new double[md.numVars];
                for (int k = 0; k < md.kBuckets; k++)
                {
                    foreach (UserMILP i in md.getV_ED())
                    {
                        if (md.Xijk.ContainsKey((j, i, k)))
                        {
                            row[md.Xijk[(j, i, k)]] += 1;
                        }
                    }
                    foreach (UserMILP i in md.getV0())
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

            /*
             * There can not be active more than on interval at time period $k$ (probably redundant, but staying here for the depot)
             */
            //tex: $  \sum_{k \in K} x_{ij}^k \leq 1,  ~\forall i \in V_{0} ~\forall j \in V_{ED}$ 
            foreach (UserMILP i in md.getV0())
            {

                foreach (UserMILP j in md.getV_ED())
                {
                    double[] row = new double[md.numVars];
                    bool any = false;
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        if (md.Xijk.ContainsKey((i, j, k)))
                        {
                            any = true;
                            row[md.Xijk[(i, j, k)]] = 1;
                        }

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


            /*
             * Selecting a CS between  i j can be 0 or 1, but it hast to be zero if $x_{ij}^k$ is zero
             */
            //tex: $   \sum_{s \in F} \sum_{k \in K} z_{isj}^k \leq \sum_{k \in K} x_{ij}^k,  ~\forall i \in V_{0} ~\forall j \in V_{ED}$ 
            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getV_ED())
                {
                    
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        double[] row = new double[md.numVars];
                        bool any = false;
                        if (md.Xijk.ContainsKey((i, j, k)))
                        {
                            any = true;
                            row[md.Xijk[(i, j, k)]] = -1;
                        }
                        foreach (UserMILP s in md.getF())
                        {
                            if (md.Zisjk.ContainsKey((i, s, j, k)))
                            {
                                any = true;
                                row[md.Zisjk[(i, s, j, k)]] = 1;
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
                            bm.eqType.Add("lowerOrEqual");
                        }
                    }
                    
                }


            }


            /*
             * For CS we have to add additional constraint as $zeta_{isj}^k$ can fall in following  time buckets and not the same time bucket as $zeta_{ij}^k$:
             */
            //tex: $   \sum_{k \in K } \overline{z}_{isj}^k = \sum_{k \in K } z_{isj}^k, ~\forall i \in V_0, \forall j \in V_{ED}, ~ i\neq j,$ 
            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getV_ED())
                {
                    foreach (UserMILP s in md.getF())
                    {
                        double[] row = new double[md.numVars];
                        bool any = false;
                        for (int k = 0; k < md.kBuckets; k++)
                        {

                            if (md.Zisjk.ContainsKey((i, s, j, k)) && md.Z_isjk.ContainsKey((i, s, j, k)))
                            {
                                any = true;
                                row[md.Z_isjk[(i, s, j, k)]] = 1;
                                row[md.Zisjk[(i, s, j, k)]] = -1;
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
                            bm.eqType.Add("equal");
                        }
                    }
                }


            }





            // Constraints on departure time at customer ($\zeta_{ij}^k$)
            //greater or equal
            //tex: $\sum_{k \in K}  \zeta_{ij}^k \geq \tau_i+s_i +   (\sum_{k \in K}x_{ij}^k-1) (l_0+gQ) , ~\forall i \in V_{0} , ~\forall j \in V_{ED} $ 

            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getV_ED())
                {
                    double[] row = new double[md.numVars];
                    bool any = false;
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        if (md.Zetaijk.ContainsKey((i, j, k)) && md.Xijk.ContainsKey((i, j, k)))
                        {
                            any = true;
                            row[md.Zetaijk[(i, j, k)]] = 1;
                            row[md.Xijk[(i, j, k)]] = -(p.depot.ltw + p.refuelRate * p.batCap);
                            //row[md.Zetaijk[(i, j, k)]] = p.getSlopeFromLinTime(i.u, j.u, k);
                            //row[md.Xijk[(i, j, k)]] = p.getSectionFromLinTime(i.u, j.u, k) + p.depot.ltw + p.refuelRate * p.batCap + i.u.serviceTime;
                        }
                    }
                    if (any)
                    {
                        row[i.serviceStartTimeVarInd] = -1;
                        if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                        {
                            Misc.errOut("All values in a row should not be zero!");
                        }
                        bm.A.Add(row);
                        bm.b.Add(i.u.serviceTime - (p.depot.ltw + p.refuelRate * p.batCap));
                        bm.eqType.Add("greaterOrEqual");
                    }
                }
            }


            // Constraints on departure time at customer ($\zeta_{ij}^k$)
            //lower or equal
            //tex: $\sum_{k \in K}  \zeta_{ij}^k \leq \tau_i+s_i +(1-\sum_{k \in K}x_{ij}^k)(l_0+gQ), ~\forall i \in V_{0} , ~\forall j \in V_{ED} $ 

            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getV_ED())
                {
                    double[] row = new double[md.numVars];
                    bool any = false;
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        if (md.Zetaijk.ContainsKey((i, j, k)) && md.Xijk.ContainsKey((i, j, k)))
                        {
                            any = true;
                            row[md.Zetaijk[(i, j, k)]] = 1;
                            row[md.Xijk[(i, j, k)]] = (p.depot.ltw + p.refuelRate * p.batCap);
                        }
                    }
                    if (any)
                    {
                        row[i.serviceStartTimeVarInd] = -1;
                        if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                        {
                            Misc.errOut("All values in a row should not be zero!");
                        }
                        bm.A.Add(row);
                        bm.b.Add(i.u.serviceTime + (p.depot.ltw + p.refuelRate * p.batCap));
                        bm.eqType.Add("lowerOrEqual");
                    }
                }
            }

            // Constraints on departure time at customer ($\zeta_{ij}^k$)
            //lower than
            //tex: $0 \leq  \sum_{k \in K}  \zeta_{ij}^k \leq \sum_{k \in K}  x_{ij}^k (l_0+gQ) , ~\forall i \in V_{0} , ~\forall j \in V_{ED} $ 

            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getV_ED())
                {
                    double[] row1 = new double[md.numVars];
                    //double[] row2 = new double[md.numVars];
                    bool any = false;
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        if (md.Zetaijk.ContainsKey((i, j, k)) && md.Xijk.ContainsKey((i, j, k)))
                        {
                            any = true;
                            row1[md.Zetaijk[(i, j, k)]] = 1;
                            row1[md.Xijk[(i, j, k)]] = -(p.depot.ltw + p.refuelRate * p.batCap);

                            //row2[md.Zetaijk[(i, j, k)]] = 1;
                        }
                    }
                    if (any)
                    {
                        if (!row1.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                        {
                            Misc.errOut("All values in a row should not be zero!");
                        }

                        //if (!row2.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                        //{
                        //    Misc.errOut("All values in a row should not be zero!");
                        //}
                        bm.A.Add(row1);
                        bm.b.Add(0);
                        bm.eqType.Add("lowerOrEqual");


                        //bm.A.Add(row2);
                        //bm.b.Add(0);
                        //bm.eqType.Add("greaterOrEqual");
                    }
                }
            }



            // Constraints on departure time at CS s ($\zeta_{isj}^k$)
            //greater or equal
            //tex: $\sum_{k \in K} \zeta_{isj}^k \geq \sum_{k \in K} \zeta_{ij}^k+ \sum_{k \in K} (\Theta_{is}^k \zeta_{ij}^k + \eta_{is}^k z_{isj}^k)+ g(Q \sum_{k \in K} z_{isj}^k -y_{isj})-(1- \sum_{k \in K}z_{isj}^k)2(l_0+gQ), \nonumber\\[1pt] ~\forall i \in V_0 , ~\forall j \in V_{ED}, ~\forall s \in F  $ 

            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getV_ED())
                {
                    foreach (UserMILP s in md.getF())
                    {
                        double[] row = new double[md.numVars];
                        bool any = false;
                        for (int k = 0; k < md.kBuckets; k++)
                        {
                            if (md.Zetaisjk.ContainsKey((i, s, j, k)) && md.Zetaijk.ContainsKey((i, j, k))
                                && md.Zisjk.ContainsKey((i, s, j, k)))
                            {
                                any = true;
                                row[md.Zetaisjk[(i, s, j, k)]] = 1;
                                row[md.Zetaijk[(i, j, k)]] = -1 - p.getSlopeFromLinTime(i.u, s.u, k);
                                row[md.Zisjk[(i, s, j, k)]] = -p.getSectionFromLinTime(i.u, s.u, k) - (p.refuelRate * p.batCap) - 2 * (p.depot.ltw + p.refuelRate * p.batCap);
                            }
                        }
                        if (any)
                        {
                            if (md.Yisj.ContainsKey((i, s, j)))
                            {
                                row[md.Yisj[(i, s, j)]] = p.refuelRate;
                                if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                                {
                                    Misc.errOut("All values in a row should not be zero!");
                                }
                                bm.A.Add(row);
                                bm.b.Add(-2 * (p.depot.ltw + p.refuelRate * p.batCap));
                                bm.eqType.Add("greaterOrEqual");
                            }
                        }
                    }
                }
            }


            // Constraints on departure time at CS s ($\zeta_{isj}^k$)
            //lower or equal
            //tex: $\sum_{k \in K} \zeta_{isj}^k \leq \sum_{k \in K} \zeta_{ij}^k+ \sum_{k \in K} (\Theta_{is}^k \zeta_{ij}^k + \eta_{is}^k z_{isj}^k)+ g(Q \sum_{k \in K} z_{isj}^k -y_{isj})+(1-\sum_{k \in K}z_{isj}^k)(l_0+gQ), \nonumber\\[1pt] ~\forall i \in V_0 , ~\forall j \in V_{ED}, ~\forall s \in F   $ 

            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getV_ED())
                {
                    foreach (UserMILP s in md.getF())
                    {
                        double[] row = new double[md.numVars];
                        bool any = false;
                        for (int k = 0; k < md.kBuckets; k++)
                        {
                            if (md.Zetaisjk.ContainsKey((i, s, j, k)) && md.Zetaijk.ContainsKey((i, j, k))
                                && md.Zisjk.ContainsKey((i, s, j, k)))
                            {
                                any = true;
                                row[md.Zetaisjk[(i, s, j, k)]] = 1;
                                row[md.Zetaijk[(i, j, k)]] = -1 - p.getSlopeFromLinTime(i.u, s.u, k);
                                row[md.Zisjk[(i, s, j, k)]] = -p.getSectionFromLinTime(i.u, s.u, k) - (p.refuelRate * p.batCap) + (p.depot.ltw + p.refuelRate * p.batCap);
                            }
                        }
                        if (any)
                        {
                            if (md.Yisj.ContainsKey((i, s, j)))
                            {
                                row[md.Yisj[(i, s, j)]] = p.refuelRate;
                                if (!row.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                                {
                                    Misc.errOut("All values in a row should not be zero!");
                                }
                                bm.A.Add(row);
                                bm.b.Add((p.depot.ltw + p.refuelRate * p.batCap));
                                bm.eqType.Add("lowerOrEqual");
                            }
                        }
                    }
                }
            }

            // Constraints on departure time at CS s ($\zeta_{isj}^k$)
            //lower than
            //tex: $ 0 \leq \sum_{k \in K}  \zeta_{isj}^k \leq  \sum_{k \in K}  z_{isj}^k (l_0+gQ)  , ~\forall i \in V_{0} , ~\forall j \in V_{ED} ,~\forall s \in F   $ 

            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getV_ED())
                {
                    foreach (UserMILP s in md.getF())
                    {
                        double[] row1 = new double[md.numVars];
                        //double[] row2 = new double[md.numVars];
                        bool any = false;
                        for (int k = 0; k < md.kBuckets; k++)
                        {
                            if (md.Zisjk.ContainsKey((i, s, j, k)) && md.Zetaisjk.ContainsKey((i, s, j, k)))
                            {
                                any = true;
                                row1[md.Zetaisjk[(i, s, j, k)]] = 1;
                                row1[md.Zisjk[(i, s, j, k)]] = -(p.depot.ltw + p.refuelRate * p.batCap);

                                //row2[md.Zetaisjk[(i, s, j, k)]] = 1;
                            }
                        }
                        if (any)
                        {
                            if (!row1.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                            {
                                Misc.errOut("All values in a row should not be zero!");
                            }
                            bm.A.Add(row1);
                            bm.b.Add(0);
                            bm.eqType.Add("lowerOrEqual");

                            //if (!row2.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                            //{
                            //    Misc.errOut("All values in a row should not be zero!");
                            //}
                            //bm.A.Add(row2);
                            //bm.b.Add(0);
                            //bm.eqType.Add("greaterOrEqual");
                        }
                    }
                }
            }


            // Delta ($\Delta$) is additional time spent going isj instead of ij:
            //greater or equal
            //tex: $\Delta_{isj} \geq \sum_{k \in K} \zeta_{isj}^k - \sum_{k \in K} \zeta_{ij}^k + \sum_{k \in K} ({\Theta}_{sj}^k \zeta_{isj}^k + {\eta}_{sj}^k \overline{z}_{isj}^k) - \sum_{k \in K} (\Theta_{ij}^k \zeta_{ij}^k + \eta_{ij}^k z_{isj}^k),~\forall i \in V_{ 0} , ~\forall j \in V_{ ED} ,~\forall s \in F    $ 

            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getV_ED())
                {
                    foreach (UserMILP s in md.getF())
                    {
                        double[] row = new double[md.numVars];
                        bool any = false;
                        for (int k = 0; k < md.kBuckets; k++)
                        {
                            if (md.Zetaisjk.ContainsKey((i, s, j, k)) && md.Zetaijk.ContainsKey((i, j, k))
                                && md.Zisjk.ContainsKey((i, s, j, k)) && md.Z_isjk.ContainsKey((i, s, j, k)))
                            {
                                any = true;
                                row[md.Zetaisjk[(i, s, j, k)]] = -1 - p.getSlopeFromLinTime(s.u, j.u, k);
                                row[md.Zetaijk[(i, j, k)]] = 1 + p.getSlopeFromLinTime(i.u, j.u, k);
                                row[md.Zisjk[(i, s, j, k)]] = p.getSectionFromLinTime(i.u, j.u, k);
                                row[md.Z_isjk[(i, s, j, k)]] = -p.getSectionFromLinTime(s.u, j.u, k);
                            }
                        }
                        if (any)
                        {
                            if (md.Deltaisj.ContainsKey((i, s, j)))
                            {
                                row[md.Deltaisj[(i, s, j)]] = 1;
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
                }
            }


            // Delta ($\Delta$) is additional time spent going isj instead of ij:
            //lower or equal
            //tex: $\Delta_{isj} \leq \sum_{k \in K} \zeta_{isj}^k - \sum_{k \in K} \zeta_{ij}^k + \sum_{k \in K} ({\Theta}_{sj}^k \zeta_{isj}^k + {\eta}_{sj}^k \overline{z}_{isj}^k) - \sum_{k \in K} (\Theta_{ij}^k \zeta_{ij}^k + \eta_{ij}^k z_{isj}^k)+(1- \sum_{k \in K}z_{isj}^k)(l_0+gQ), ~\forall i \in V_{0} , ~\forall j \in V_{ED} ,~\forall s \in F $ 

            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getV_ED())
                {
                    foreach (UserMILP s in md.getF())
                    {
                        double[] row = new double[md.numVars];
                        bool any = false;
                        for (int k = 0; k < md.kBuckets; k++)
                        {
                            if (md.Zetaisjk.ContainsKey((i, s, j, k)) && md.Zetaijk.ContainsKey((i, j, k))
                                && md.Zisjk.ContainsKey((i, s, j, k)) && md.Z_isjk.ContainsKey((i, s, j, k)))
                            {
                                any = true;
                                row[md.Zetaisjk[(i, s, j, k)]] = -1 - p.getSlopeFromLinTime(s.u, j.u, k);
                                row[md.Zetaijk[(i, j, k)]] = 1 + p.getSlopeFromLinTime(i.u, j.u, k);
                                row[md.Zisjk[(i, s, j, k)]] = p.getSectionFromLinTime(i.u, j.u, k) + (p.depot.ltw + p.refuelRate * p.batCap);
                                row[md.Z_isjk[(i, s, j, k)]] = -p.getSectionFromLinTime(s.u, j.u, k);
                            }
                        }
                        if (any)
                        {
                            if (md.Deltaisj.ContainsKey((i, s, j)))
                            {
                                row[md.Deltaisj[(i, s, j)]] = 1;
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
                }
            }


            // Delta ($\Delta$) is additional time spent going isj instead of ij:
            //limitation
            //tex: $ 0 \leq \Delta_{isj} \leq \sum_{k \in K} z_{isj}^k (l_0+gQ), ~\forall i \in V_{0} , ~\forall j \in V_{ED} ,~\forall s \in F $ 

            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getV_ED())
                {
                    foreach (UserMILP s in md.getF())
                    {
                        double[] row1 = new double[md.numVars];
                        //double[] row2 = new double[md.numVars];
                        bool any = false;
                        for (int k = 0; k < md.kBuckets; k++)
                        {
                            if (md.Zisjk.ContainsKey((i, s, j, k)))
                            {
                                any = true;
                                row1[md.Zisjk[(i, s, j, k)]] = -(p.depot.ltw + p.refuelRate * p.batCap);

                            }
                        }
                        if (any)
                        {
                            if (md.Deltaisj.ContainsKey((i, s, j)))
                            {
                                row1[md.Deltaisj[(i, s, j)]] = 1;
                                if (!row1.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                                {
                                    Misc.errOut("All values in a row should not be zero!");
                                }
                                bm.A.Add(row1);
                                bm.b.Add(0);
                                bm.eqType.Add("lowerOrEqual");


                                //row2[md.Deltaisj[(i, s, j)]] = 1;
                                //if (!row2.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                                //{
                                //    Misc.errOut("All values in a row should not be zero!");
                                //}
                                //bm.A.Add(row2);
                                //bm.b.Add(0);
                               // bm.eqType.Add("greaterOrEqual");
                            }
                        }
                    }
                }
            }

            // Service start time at customer j ($\tau_j$):
            //LEQ
            //tex: $ \tau_i+s_i \sum_{k \in K}x_{ij}^k+\sum_{k \in K} (\Theta_{ij}^k \zeta_{ij}^k + \eta_{ij}^k x_{ij}^k) + \sum_{s \in F} \Delta_{isj} -(l_0+gQ)(1-\sum_{k \in K} x_{ij}^k) \leq \tau_j, \forall i \in V_0, \forall j \in V_{ ED}~i\neq j $ 

            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getV_ED())
                {
                    double[] row = new double[md.numVars];
                    bool any = false;
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        if (md.Xijk.ContainsKey((i, j, k)) &&
                        md.Zetaijk.ContainsKey((i, j, k)))
                        {
                            any = true;
                            row[md.Zetaijk[(i, j, k)]] = p.getSlopeFromLinTime(i.u, j.u, k);
                            row[md.Xijk[(i, j, k)]] = i.u.serviceTime + p.getSectionFromLinTime(i.u, j.u, k) + (p.depot.ltw + p.refuelRate * p.batCap);
                        }
                    }

                    foreach (UserMILP s in md.getF())
                    {
                        if (md.Deltaisj.ContainsKey((i, s, j)))
                        {
                            any = true;
                            row[md.Deltaisj[(i, s, j)]] = 1;
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
                        bm.b.Add((p.depot.ltw + p.refuelRate * p.batCap));
                        bm.eqType.Add("lowerOrEqual");
                    }
                }
            }


            // Service start time at customer j ($\tau_j$):
            //GEQ
            //tex: $ \tau_i+s_i \sum_{k \in K}x_{ij}^k+\sum_{k \in K} (\Theta_{ij}^k \zeta_{ij}^k + \eta_{ij}^k x_{ij}^k) + \sum_{s \in F} \Delta_{isj} +(l_0+gQ)(1-\sum_{k \in K} x_{ij}^k) +(l_0+gQ)b_{ij} \geq \tau_j, \forall i \in V_0, \forall j \in V_{ED}~ i\neq j  $ 

            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getV_ED())
                {
                    double[] row = new double[md.numVars];
                    bool any = false;
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        if (md.Xijk.ContainsKey((i, j, k)) &&
                        md.Zetaijk.ContainsKey((i, j, k)))
                        {
                            any = true;
                            row[md.Zetaijk[(i, j, k)]] = p.getSlopeFromLinTime(i.u, j.u, k);
                            row[md.Xijk[(i, j, k)]] = i.u.serviceTime + p.getSectionFromLinTime(i.u, j.u, k) - (p.depot.ltw + p.refuelRate * p.batCap);
                        }
                    }
                    if (md.Bij.ContainsKey((i, j)))
                    {
                        any = true;
                        row[md.Bij[(i, j)]] = p.depot.ltw + p.refuelRate * p.batCap;
                    }

                    foreach (UserMILP s in md.getF())
                    {
                        if (md.Deltaisj.ContainsKey((i, s, j)))
                        {
                            any = true;
                            row[md.Deltaisj[(i, s, j)]] = 1;
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
                        bm.b.Add(-(p.depot.ltw + p.refuelRate * p.batCap));
                        bm.eqType.Add("greaterOrEqual");
                    }
                }
            }

            // Service start time at customer j ($\tau_j$):
            //Limit on early time window
            //tex: $ \tau_j \leq e_j+2(l_0+gQ)-b_{ij}(l_0+gQ) -\sum_{k \in K}x_{ij}^k(l_0+gQ),~ \forall i \in V_0, \forall j \in V_{ED} ,~ i\neq j  $ 

            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getV_ED())
                {
                    double[] row = new double[md.numVars];
                    bool any = false;
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        if (md.Xijk.ContainsKey((i, j, k)))
                        {
                            any = true;
                            row[md.Xijk[(i, j, k)]] = (p.depot.ltw + p.refuelRate * p.batCap);
                        }
                    }
                    if (md.Bij.ContainsKey((i, j)))
                    {
                        any = true;
                        row[md.Bij[(i, j)]] = (p.depot.ltw + p.refuelRate * p.batCap);
                    }

                    if (any)
                    {
                        row[j.serviceStartTimeVarInd] = 1;
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











            // Equation for ensuring that each departure time of customer/CS is within its appropriate breakpointes
            // determined by breakpoints of time buckets and latest departure times at customers/CSs
            //tex: $x_{ij}^k \max{ (w_k,e_i+s_i)} \leq \zeta_{ij}^k \leq \min{ (w_{k+1},l_i+s_i)}x_{ij}^k, ~\forall i \in V_0, \forall j \in V_{ED},~ i\neq j, \forall k \in K$

            //tex: $\overline{z}_{isj}^k \max{ (w_k,e_i+s_i+\Theta_{ij}^k(e_i+s_i)}+\mu_{ij}^k )\leq \zeta_{isj}^k \leq \min{ (w_{k+1},l_i+gQ)} \overline{z}_{isj}^k, \nonumber\\[1pt] ~\forall i \in V_{0} , ~\forall j \in V_{ED} ,~\forall s \in F , \forall k \in K$
            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getV_ED())
                {
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        if (md.Zetaijk.ContainsKey((i, j, k)) && md.Xijk.ContainsKey((i, j, k)))
                        {
                            double[] row1 = new double[md.numVars];
                            Tuple<double, double> boundaries = p.getBoundaries(k);
                            row1[md.Xijk[(i, j, k)]] = Math.Max(boundaries.Item1, i.u.etw + i.u.serviceTime);
                            row1[md.Zetaijk[(i, j, k)]] = -1;
                            if (!row1.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                            {
                                Misc.errOut("All values in a row should not be zero!");
                            }
                            bm.A.Add(row1);
                            bm.b.Add(0);
                            bm.eqType.Add("lowerOrEqual");

                            double[] row2 = new double[md.numVars];
                            row2[md.Xijk[(i, j, k)]] = -Math.Min(boundaries.Item2, i.u.ltw + i.u.serviceTime);
                            row2[md.Zetaijk[(i, j, k)]] = 1;
                            if (!row2.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                            {
                                Misc.errOut("All values in a row should not be zero!");
                            }
                            bm.A.Add(row2);
                            bm.b.Add(0);
                            bm.eqType.Add("lowerOrEqual");
                        }

                        foreach (UserMILP s in md.getF())
                        {
                            if (md.Zetaisjk.ContainsKey((i, s, j, k)) && md.Z_isjk.ContainsKey((i, s, j, k)))
                            {
                                double[] row3 = new double[md.numVars];
                                Tuple<double, double> boundaries = p.getBoundaries(k);
                                row3[md.Z_isjk[(i, s, j, k)]] = Math.Max(boundaries.Item1, i.u.etw + i.u.serviceTime + p.getSlopeFromLinTime(i.u, s.u, k) * (i.u.etw + i.u.serviceTime) + p.getSectionFromLinTime(i.u, s.u, k));
                                //row3[md.Z_isjk[(i, s, j, k)]] = Math.Max(boundaries.Item1, 0);
                                row3[md.Zetaisjk[(i, s, j, k)]] = -1;
                                if (!row3.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                                {
                                    Misc.errOut("All values in a row should not be zero!");
                                }
                                bm.A.Add(row3);
                                bm.b.Add(0);
                                bm.eqType.Add("lowerOrEqual");

                                double[] row4 = new double[md.numVars];
                                row4[md.Z_isjk[(i, s, j, k)]] = -Math.Min(boundaries.Item2, s.u.ltw + p.refuelRate * p.batCap);

                                //row4[md.Z_isjk[(i, s, j, k)]] = -Math.Min(boundaries.Item2, double.MaxValue);
                                row4[md.Zetaisjk[(i, s, j, k)]] = 1;
                                if (!row4.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                                {
                                    Misc.errOut("All values in a row should not be zero!");
                                }
                                bm.A.Add(row4);
                                bm.b.Add(0);
                                bm.eqType.Add("lowerOrEqual");
                            }
                        }
                    }
                }
            }

            // Equation for reamining load capcity flow conservation
            //tex: $u_j \leq u_i- \sum_{k \in K}x_{ij}^k (q_i+C)+C, ~\forall i \in V_0, \forall j \in V_{ED},~ i\neq j$ 

            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getV_ED())
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

            // Equation for reamining battery capcity flow conservation (energy) for CSs
            //LEQ
            //tex: $0 \leq y_{isj} \leq y_i-\sum_{k \in K} z_{isj}^k e_{is}^k+Q(1- \sum_{k \in K} z_{isj}^k), ~\forall i \in V_0, \forall j \in V_{ED},~ i\neq j,  ~\forall s \in F$ 
            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getV_ED())
                {
                    foreach (UserMILP s in md.getF())
                    {
                        double[] row1 = new double[md.numVars];
                        row1[i.restEnergyVarInd] = -1;
                        bool any = false;

                        if (md.Yisj.ContainsKey((i, s, j)))
                        {

                            row1[md.Yisj[(i, s, j)]] = 1;

                            for (int k = 0; k < md.kBuckets; k++)
                            {
                                if (md.Zisjk.ContainsKey((i, s, j, k)))
                                {
                                    any = true;
                                    row1[md.Zisjk[(i, s, j, k)]] = p.ener(i.u, s.u, k) + p.batCap;
                                }
                            }
                        }
                        if (any)
                        {
                            if (!row1.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                            {
                                Misc.errOut("All values in a row should not be zero!");
                            }
                            bm.A.Add(row1);
                            bm.b.Add(p.batCap);
                            bm.eqType.Add("lowerOrEqual");
                        }
                    }
                }
            }

            // Equation for reamining battery capcity flow conservation (energy) for CSs
            //GEQ
            //tex: $y_{isj} \geq y_i- \sum_{k \in K} z_{isj}^k e_{is}^k+Q(\sum_{k \in K} z_{isj}^k -1), ~\forall i \in V_0, \forall j \in V_{ED},~ i\neq j, ~\forall s \in F$ 
            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getV_ED())
                {
                    foreach (UserMILP s in md.getF())
                    {
                        double[] row1 = new double[md.numVars];
                        row1[i.restEnergyVarInd] = -1;
                        bool any = false;

                        if (md.Yisj.ContainsKey((i, s, j)))
                        {

                            row1[md.Yisj[(i, s, j)]] = 1;

                            for (int k = 0; k < md.kBuckets; k++)
                            {
                                any = true;
                                if (md.Zisjk.ContainsKey((i, s, j, k)))
                                {
                                    row1[md.Zisjk[(i, s, j, k)]] = p.ener(i.u, s.u, k) - p.batCap;
                                }
                            }
                        }
                        if (any)
                        {
                            if (!row1.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                            {
                                Misc.errOut("All values in a row should not be zero!");
                            }
                            bm.A.Add(row1);
                            bm.b.Add(-p.batCap);
                            bm.eqType.Add("greaterOrEqual");
                        }
                    }
                }
            }

            // Equation for reamining battery capcity flow conservation (energy) for CSs
            //Limiting
            //tex: $y_{isj} \leq Q\sum_{k \in K} z_{isj}^k, ~\forall i \in V_0, \forall j \in V_{ED},~ i\neq j, \forall s \in F$ 
            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getV_ED())
                {
                    foreach (UserMILP s in md.getF())
                    {
                        double[] row1 = new double[md.numVars];
                        bool any = false;

                        if (md.Yisj.ContainsKey((i, s, j)))
                        {

                            row1[md.Yisj[(i, s, j)]] = 1;

                            for (int k = 0; k < md.kBuckets; k++)
                            {
                                if (md.Zisjk.ContainsKey((i, s, j, k)))
                                {
                                    any = true;
                                    row1[md.Zisjk[(i, s, j, k)]] = -p.batCap;
                                }
                            }
                        }
                        if (any)
                        {
                            if (!row1.Any(v => !Misc.EqualDoubleValues(v, 0, p.doublePrecision)))
                            {
                                Misc.errOut("All values in a row should not be zero!");
                            }
                            bm.A.Add(row1);
                            bm.b.Add(0);
                            bm.eqType.Add("lowerOrEqual");
                        }
                    }
                }
            }


            // Equation for reamining battery capcity flow conservation (energy) for customers
            //LEQ
            //tex: $y_j \leq y_i- \sum_{k \in K}x_{ij}^k e_{ij}^k+Q \sum_{s \in F} \sum_{k \in K} z_{isj}^k + Q(1- \sum_{k \in K} x_{ij}^k), ~\forall i \in V_0, \forall j \in V_{ED},~ i\neq j$ 
            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getV_ED())
                {
                    double[] row = new double[md.numVars];
                    bool any = false;
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        if (md.Xijk.ContainsKey((i, j, k)))
                        {
                            row[md.Xijk[(i, j, k)]] = p.ener(i.u, j.u, k) + p.batCap;
                            any = true;
                        }
                        foreach (UserMILP s in md.getF())
                        {
                            if (md.Zisjk.ContainsKey((i, s, j, k)))
                            {
                                row[md.Zisjk[(i, s, j, k)]] = -p.batCap;
                                any = true;
                            }
                        }
                    }
                    if (any)
                    {
                        row[j.restEnergyVarInd] = 1;
                        row[i.restEnergyVarInd] = -1;
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
            //GEQ
            //tex: $y_j \geq y_i- \sum_{k \in K}x_{ij}^k e_{ij}^k-Q \sum_{s \in F} \sum_{k \in K} z_{isj}^k - Q(1- \sum_{k \in K} x_{ij}^k), ~\forall i \in V_0, \forall j \in V_{ED},~ i\neq j$ 
            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getV_ED())
                {
                    double[] row = new double[md.numVars];
                    bool any = false;
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        if (md.Xijk.ContainsKey((i, j, k)))
                        {
                            row[md.Xijk[(i, j, k)]] = p.ener(i.u, j.u, k) - p.batCap;
                            any = true;
                        }
                        foreach (UserMILP s in md.getF())
                        {
                            if (md.Zisjk.ContainsKey((i, s, j, k)))
                            {
                                row[md.Zisjk[(i, s, j, k)]] = p.batCap;
                                any = true;
                            }
                        }
                    }
                    if (any)
                    {
                        row[j.restEnergyVarInd] = 1;
                        row[i.restEnergyVarInd] = -1;
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

            // Equation for reamining battery capcity flow conservation (energy) for customers after CS
            //LEQ
            //tex: $y_j \leq Q - \sum_{s \in F} \sum_{k \in K} \overline{z}_{isj}^k e_{sj}^k, ~\forall i \in V_0, \forall j \in V_{ED},~ i\neq j$ 
            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getV_ED())
                {
                    double[] row = new double[md.numVars];
                    bool any = false;
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        foreach (UserMILP s in md.getF())
                        {
                            if (md.Z_isjk.ContainsKey((i, s, j, k)))
                            {
                                row[md.Z_isjk[(i, s, j, k)]] = p.ener(s.u, j.u, k);
                                any = true;
                            }
                        }
                    }
                    if (any)
                    {
                        row[j.restEnergyVarInd] = 1;
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




            // Equation for reamining battery capcity flow conservation (energy) for customers after CS
            //GEQ
            //tex: $y_j \geq Q\sum_{s \in F} \sum_{k \in K}z_{isj}^k - \sum_{s \in F} \sum_{k \in K} \overline{z}_{isj}^k e_{sj}^k , ~\forall i \in V_0, \forall j \in V_{ED},~ i\neq j$ 
            foreach (UserMILP i in md.getV0())
            {
                foreach (UserMILP j in md.getV_ED())
                {
                    double[] row = new double[md.numVars];
                    bool any = false;
                    for (int k = 0; k < md.kBuckets; k++)
                    {
                        foreach (UserMILP s in md.getF())
                        {
                            if (md.Zisjk.ContainsKey((i, s, j, k)) && md.Z_isjk.ContainsKey((i, s, j, k)))
                            {
                                row[md.Zisjk[(i, s, j, k)]] = -p.batCap;
                                row[md.Z_isjk[(i, s, j, k)]] = p.ener(s.u, j.u, k);
                                any = true;
                            }
                        }
                    }
                    if (any)
                    {
                        row[j.restEnergyVarInd] = 1;
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
