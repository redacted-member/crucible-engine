// SPDX-License-Identifier: MIT
// Strips the TypeScript types from src/*.ts (Node 22.6+ built-in; the sources use erasable syntax only) and copies the
// static files into dist/. Usage: node build.mjs   (the wasm is built separately, see README.md)
import { copyFileSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { stripTypeScriptTypes } from 'node:module';

mkdirSync('dist', { recursive: true });
for (const file of ['index.html', 'style.css', 'engine-worklet.js']) copyFileSync(`src/${file}`, `dist/${file}`);
writeFileSync('dist/main.js', stripTypeScriptTypes(readFileSync('src/main.ts', 'utf8')));
