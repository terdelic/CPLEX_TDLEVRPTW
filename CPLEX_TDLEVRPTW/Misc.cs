using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Net;
using System.Net.Mail;

namespace CPLEX_TDTSPTW
{
    public static class Misc
    {
        //Method to print out the error, and pause the program
        public static void errOut(string str)
        {
            Console.WriteLine(str);
            Console.ReadKey();
        }
        //Method to check wether the two double values are the "same" within the resolution precision
        public static bool EqualDoubleValues(double val1, double val2, double precision)
        {
            if (Math.Abs(val1 - val2) <= precision)
            {
                return true;
            }
            return false;
        }
        /*Send an email to myself, when the optimization is finsihed*/
        //This will not be published in open source version
        public static void SendEmail(string text)
        {
            MailAddress from = new MailAddress("erdelictomislav@gmail.com");
            MailAddress to = new MailAddress("terdelic@fpz.hr");
            MailMessage message = new MailMessage(from, to);
            message.Subject = "[TDEVRPTW] - Notfication";
            message.Body = text;
            message.BodyEncoding = System.Text.Encoding.UTF8;
            message.IsBodyHtml = true;

            SmtpClient client = new SmtpClient("smtp.gmail.com", 587);
            client.UseDefaultCredentials = false;
            client.EnableSsl = true;
            //The password was acquired from setting to allow Less secure apps in gmail settings
            client.Credentials = new NetworkCredential("drscnotification@gmail.com", "pnnfnxzwzeqfjnhm");
            try
            {
                client.Send(message);
                Console.WriteLine("Your Message Send Successfully.");
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
            }
            finally
            {

            }
        }
    }
}
