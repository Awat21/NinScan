using System;
using System.Collections.Generic;
using System.IO;
using NinScan.Core.Infrastructure;
using NinScan.Core.Mount;

namespace NinScan.Driver
{
    /// <summary>
    /// The one mount engine shared by every client of this server. Each N.I.N.A., SharpCap or PHD2
    /// connection gets its own Telescope object; the hardware stays connected while any of them is.
    /// </summary>
    internal static class SharedMount
    {
        private static readonly object Lock = new object();
        private static readonly HashSet<Guid> ConnectedClients = new HashSet<Guid>();
        private static AltAzMount _engine;
        private static FileLog _log;

        public static string DataDirectory =>
            Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "NinScan");

        public static string SettingsPath => Path.Combine(DataDirectory, "settings.json");

        public static string LogDirectory => Path.Combine(DataDirectory, "logs");

        public static FileLog Log
        {
            get
            {
                lock (Lock)
                {
                    if (_log == null) _log = new FileLog(LogDirectory, false);
                    return _log;
                }
            }
        }

        public static AltAzMount Engine
        {
            get
            {
                lock (Lock)
                {
                    if (_engine == null)
                    {
                        var settings = MountSettings.Load(SettingsPath);
                        Log.DebugEnabled = settings.DebugLogging;
                        Log.Info("NinScan " + typeof(SharedMount).Assembly.GetName().Version + " starting; settings from " + SettingsPath);
                        _engine = new AltAzMount(settings, SystemClock.Instance, Log, DataDirectory);
                    }
                    return _engine;
                }
            }
        }

        public static bool IsConnected(Guid client)
        {
            lock (Lock) return ConnectedClients.Contains(client) && Engine.Connected;
        }

        public static void Connect(Guid client)
        {
            lock (Lock)
            {
                if (!Engine.Connected) Engine.Connect();
                ConnectedClients.Add(client);
                Log.Info("Client connected (" + ConnectedClients.Count + " connected).");
            }
        }

        public static void Disconnect(Guid client)
        {
            lock (Lock)
            {
                if (!ConnectedClients.Remove(client)) return;
                Log.Info("Client disconnected (" + ConnectedClients.Count + " still connected).");
                if (ConnectedClients.Count == 0) Engine.Disconnect();
            }
        }

        public static void SaveSettings(MountSettings settings)
        {
            settings.Validate();
            lock (Lock)
            {
                settings.Save(SettingsPath);
                Engine.UpdateSettings(settings);
                Log.DebugEnabled = settings.DebugLogging;
                Log.Info("Settings saved.");
            }
        }

        public static void Shutdown()
        {
            lock (Lock)
            {
                ConnectedClients.Clear();
                if (_engine != null && _engine.Connected) _engine.Disconnect();
            }
        }
    }
}
