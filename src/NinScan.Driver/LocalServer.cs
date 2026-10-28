using System;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Security.Principal;
using System.Threading;
using System.Windows.Forms;
using Microsoft.Win32;

namespace NinScan.Driver
{
    /// <summary>
    /// Out-of-process COM server. COM launches this exe with -Embedding when a client creates the
    /// driver, and it exits on its own once the last client lets go. Run with /regserver (as admin)
    /// to register, /unregserver to remove, or with no arguments to open the settings.
    /// </summary>
    internal static class LocalServer
    {
        private const string AppId = "{f2403b05-93d4-48bf-8642-f162f92eaf69}";
        private const uint CLSCTX_LOCAL_SERVER = 4;
        private const uint REGCLS_MULTIPLEUSE = 1;
        private const uint REGCLS_SUSPENDED = 4;

        private static int _objectCount;
        private static int _lockCount;
        private static bool _startedByCom;
        private static uint _cookie;

        [STAThread]
        private static int Main(string[] args)
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);

            string arg = args.Length > 0 ? args[0].TrimStart('-', '/').ToLowerInvariant() : "";
            switch (arg)
            {
                case "regserver":
                case "register":
                    return RunElevated("/regserver", Register);
                case "unregserver":
                case "unregister":
                    return RunElevated("/unregserver", Unregister);
                case "embedding":
                    _startedByCom = true;
                    break;
                case "":
                    using (var f = new SetupForm()) f.ShowDialog();
                    SharedMount.Shutdown();
                    return 0;
                default:
                    MessageBox.Show("Unknown option '" + args[0] + "'. Use /regserver, /unregserver or no option to open the settings.", "NinScan");
                    return 1;
            }

            try
            {
                var factory = new ClassFactory(typeof(Telescope));
                var clsid = typeof(Telescope).GUID;
                int hr = CoRegisterClassObject(ref clsid, factory, CLSCTX_LOCAL_SERVER, REGCLS_MULTIPLEUSE | REGCLS_SUSPENDED, out _cookie);
                if (hr != 0) throw new COMException("CoRegisterClassObject failed", hr);
                CoResumeClassObjects();
                SharedMount.Log.Info("COM server started (pid " + Process.GetCurrentProcess().Id + ").");

                using (var context = new ServerContext())
                {
                    Application.Run(context);
                }
            }
            catch (Exception ex)
            {
                SharedMount.Log.Error("COM server failed", ex);
                return 1;
            }
            finally
            {
                if (_cookie != 0) CoRevokeClassObject(_cookie);
                SharedMount.Shutdown();
                SharedMount.Log.Info("COM server stopped.");
            }
            return 0;
        }

        public static void ObjectCreated() => Interlocked.Increment(ref _objectCount);

        public static void ObjectReleased() => Interlocked.Decrement(ref _objectCount);

        public static void LockServer(bool fLock)
        {
            if (fLock) Interlocked.Increment(ref _lockCount);
            else Interlocked.Decrement(ref _lockCount);
        }

        /// <summary>Message loop for the server; checks every few seconds whether anyone still needs it.</summary>
        private sealed class ServerContext : ApplicationContext
        {
            private readonly System.Windows.Forms.Timer _timer = new System.Windows.Forms.Timer { Interval = 3000 };
            private int _idleChecks;

            public ServerContext()
            {
                _timer.Tick += (s, e) =>
                {
                    // Clients release objects through COM; the CLR only notices on collection.
                    GC.Collect();
                    GC.WaitForPendingFinalizers();
                    if (_startedByCom && Volatile.Read(ref _objectCount) <= 0 && Volatile.Read(ref _lockCount) <= 0)
                    {
                        if (++_idleChecks >= 2) ExitThread();
                    }
                    else
                    {
                        _idleChecks = 0;
                    }
                };
                _timer.Start();
            }

            protected override void Dispose(bool disposing)
            {
                if (disposing) _timer.Dispose();
                base.Dispose(disposing);
            }
        }

        // ------------------------------------------------------------------ registration

        private static int RunElevated(string argument, Action action)
        {
            if (!IsAdministrator())
            {
                try
                {
                    var p = Process.Start(new ProcessStartInfo(Application.ExecutablePath, argument) { Verb = "runas", UseShellExecute = true });
                    p?.WaitForExit();
                    return p?.ExitCode ?? 1;
                }
                catch (System.ComponentModel.Win32Exception)
                {
                    MessageBox.Show("NinScan needs administrator rights to " + (argument == "/regserver" ? "register" : "unregister") + " the driver.", "NinScan");
                    return 1;
                }
            }
            try
            {
                action();
                return 0;
            }
            catch (Exception ex)
            {
                MessageBox.Show("Failed: " + ex.Message, "NinScan");
                return 1;
            }
        }

        private static bool IsAdministrator()
        {
            using (var id = WindowsIdentity.GetCurrent())
            {
                return new WindowsPrincipal(id).IsInRole(WindowsBuiltInRole.Administrator);
            }
        }

        private static void Register()
        {
            var type = typeof(Telescope);
            string clsid = type.GUID.ToString("B");
            string progId = Telescope.ProgIdValue;
            string exe = Application.ExecutablePath;
            string exeName = System.IO.Path.GetFileName(exe);

            // Write both registry views so 32-bit and 64-bit clients can both find the server.
            foreach (var view in new[] { RegistryView.Registry64, RegistryView.Registry32 })
            {
                using (var root = RegistryKey.OpenBaseKey(RegistryHive.ClassesRoot, view))
                {
                    using (var k = root.CreateSubKey("AppID\\" + AppId))
                    {
                        k.SetValue(null, "NinScan");
                        k.SetValue("AppID", AppId);
                        k.SetValue("AuthenticationLevel", 1, RegistryValueKind.DWord);
                    }
                    using (var k = root.CreateSubKey("AppID\\" + exeName)) k.SetValue("AppID", AppId);

                    using (var k = root.CreateSubKey("CLSID\\" + clsid))
                    {
                        k.SetValue(null, Telescope.ChooserName);
                        k.SetValue("AppId", AppId);
                        using (var s = k.CreateSubKey("Implemented Categories\\{62C8FE65-4EBB-45e7-B440-6E39B2CDBF29}")) { }
                        using (var s = k.CreateSubKey("ProgId")) s.SetValue(null, progId);
                        using (var s = k.CreateSubKey("Programmable")) { }
                        using (var s = k.CreateSubKey("LocalServer32")) s.SetValue(null, "\"" + exe + "\"");
                    }
                    using (var k = root.CreateSubKey(progId))
                    {
                        k.SetValue(null, Telescope.ChooserName);
                        using (var s = k.CreateSubKey("CLSID")) s.SetValue(null, clsid);
                    }
                }
            }

            ASCOM.Com.Profile.Register(ASCOM.Common.DeviceTypes.Telescope, progId, Telescope.ChooserName);
            MessageBox.Show("NinScan is registered. Pick \"" + Telescope.ChooserName + "\" in the ASCOM chooser in N.I.N.A. or SharpCap.", "NinScan");
        }

        private static void Unregister()
        {
            string clsid = typeof(Telescope).GUID.ToString("B");
            string exeName = System.IO.Path.GetFileName(Application.ExecutablePath);
            foreach (var view in new[] { RegistryView.Registry64, RegistryView.Registry32 })
            {
                using (var root = RegistryKey.OpenBaseKey(RegistryHive.ClassesRoot, view))
                {
                    root.DeleteSubKeyTree("CLSID\\" + clsid, false);
                    root.DeleteSubKeyTree(Telescope.ProgIdValue, false);
                    root.DeleteSubKeyTree("AppID\\" + AppId, false);
                    root.DeleteSubKeyTree("AppID\\" + exeName, false);
                }
            }
            try { ASCOM.Com.Profile.UnRegister(ASCOM.Common.DeviceTypes.Telescope, Telescope.ProgIdValue); } catch { }
            MessageBox.Show("NinScan has been unregistered.", "NinScan");
        }

        // ------------------------------------------------------------------ COM interop

        [DllImport("ole32.dll")]
        private static extern int CoRegisterClassObject([In] ref Guid rclsid, [MarshalAs(UnmanagedType.IUnknown)] object pUnk,
            uint dwClsContext, uint flags, out uint lpdwRegister);

        [DllImport("ole32.dll")]
        private static extern int CoResumeClassObjects();

        [DllImport("ole32.dll")]
        private static extern int CoRevokeClassObject(uint dwRegister);
    }

    /// <summary>Base for COM-served objects so the server knows when it can exit.</summary>
    [ComVisible(false)]
    public abstract class ReferenceCountedObject
    {
        protected ReferenceCountedObject() => LocalServer.ObjectCreated();
        ~ReferenceCountedObject() => LocalServer.ObjectReleased();
    }

    [ComImport]
    [Guid("00000001-0000-0000-C000-000000000046")]
    [InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
    public interface IClassFactory
    {
        [PreserveSig]
        int CreateInstance(IntPtr pUnkOuter, ref Guid riid, out IntPtr ppvObject);

        [PreserveSig]
        int LockServer(bool fLock);
    }

    [ComVisible(true)]
    [ClassInterface(ClassInterfaceType.None)]
    public sealed class ClassFactory : IClassFactory
    {
        private const int CLASS_E_NOAGGREGATION = unchecked((int)0x80040110);
        private readonly Type _type;

        public ClassFactory(Type type) => _type = type;

        public int CreateInstance(IntPtr pUnkOuter, ref Guid riid, out IntPtr ppvObject)
        {
            ppvObject = IntPtr.Zero;
            if (pUnkOuter != IntPtr.Zero) return CLASS_E_NOAGGREGATION;
            object obj = Activator.CreateInstance(_type);
            IntPtr unk = Marshal.GetIUnknownForObject(obj);
            try
            {
                return Marshal.QueryInterface(unk, ref riid, out ppvObject);
            }
            finally
            {
                Marshal.Release(unk);
            }
        }

        public int LockServer(bool fLock)
        {
            LocalServer.LockServer(fLock);
            return 0;
        }
    }
}
