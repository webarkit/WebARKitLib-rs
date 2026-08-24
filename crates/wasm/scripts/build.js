#!/usr/bin/env node
// build.js
// Detects the OS and runs the appropriate dual build script.

const { execFileSync } = require('child_process');
const path = require('path');

const scriptsDir = __dirname;

try {
  if (process.platform === 'win32') {
    // Windows: ensure cargo bin is in PATH and run the PowerShell script
    const cargoBin = path.join(process.env.USERPROFILE || '', '.cargo', 'bin');
    const pathKey = Object.keys(process.env).find(k => k.toLowerCase() === 'path') || 'PATH';
    const env = { ...process.env, [pathKey]: `${cargoBin};${process.env[pathKey] || ''}` };
    const script = path.join(scriptsDir, 'build-dual.ps1');
    execFileSync('powershell.exe', ['-ExecutionPolicy', 'RemoteSigned', '-File', script], { stdio: 'inherit', env });
  } else {
    // Linux / macOS: run the bash script
    const script = path.join(scriptsDir, 'build-dual.sh');
    execFileSync('bash', [script], { stdio: 'inherit' });
  }
} catch (err) {
  const scriptName = process.platform === 'win32' ? 'build-dual.ps1' : 'build-dual.sh';
  console.error(`Error: Failed to execute ${scriptName}.`);
  console.error(err.message);
  process.exit(1);
}
