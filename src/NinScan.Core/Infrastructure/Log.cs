using System;
using System.IO;

namespace NinScan.Core.Infrastructure
{
    public interface ILog
    {
        void Info(string message);
        void Warn(string message);
        void Error(string message, Exception ex = null);
        /// <summary>Very chatty output such as every serial command. Only written when enabled.</summary>
        void Debug(string message);
    }

    public sealed class NullLog : ILog
    {
        public static readonly NullLog Instance = new NullLog();
        public void Info(string message) { }
        public void Warn(string message) { }
        public void Error(string message, Exception ex = null) { }
        public void Debug(string message) { }
    }

    /// <summary>Appends timestamped lines to a daily log file. Safe to call from any thread.</summary>
    public sealed class FileLog : ILog
    {
        private readonly object _lock = new object();
        private readonly string _directory;

        public FileLog(string directory, bool debugEnabled)
        {
            _directory = directory;
            DebugEnabled = debugEnabled;
            Directory.CreateDirectory(directory);
        }

        public bool DebugEnabled { get; set; }

        public string CurrentPath => Path.Combine(_directory, "NinScan-" + DateTime.Now.ToString("yyyy-MM-dd") + ".log");

        public void Info(string message) => Write("INFO ", message);
        public void Warn(string message) => Write("WARN ", message);
        public void Error(string message, Exception ex = null) => Write("ERROR", ex == null ? message : message + " :: " + ex);
        public void Debug(string message)
        {
            if (DebugEnabled) Write("DEBUG", message);
        }

        private void Write(string level, string message)
        {
            try
            {
                lock (_lock)
                {
                    File.AppendAllText(CurrentPath,
                        DateTime.Now.ToString("HH:mm:ss.fff") + " " + level + " " + message + Environment.NewLine);
                }
            }
            catch
            {
                // Logging must never take the driver down.
            }
        }
    }
}
