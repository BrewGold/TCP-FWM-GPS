# Programa FWM – Instaladores Dynatest

Esta carpeta documenta la instalación del software Dynatest necesario para el FWM.

## Instaladores requeridos

| Orden | Programa | Versión |
|-------|----------|---------|
| 1 | Dynatest Data Collection Prerequisites Installer | 2.1.0.246 |
| 2 | Dynatest Data Collection Installer | 2.4.1 |

## Ubicación de los instaladores

Los instaladores **no se versionan** en este repositorio. **No subas ejecutables (`.exe`, `.msi`) al repo.**
Están guardados localmente en la carpeta **Descargas** del equipo (por ejemplo `C:\Users\<USUARIO>\Downloads\`).
Si no los tienes, solicítalos al proveedor/responsable del proyecto y guárdalos en Descargas.

## Orden de instalación recomendado

1. Cierra cualquier instancia de Dynatest Data Collection.
2. Abre **Descargas** y ejecuta como administrador (clic derecho → *Ejecutar como administrador*)
   `Dynatest Data Collection Prerequisites Installer-2.1.0.246`. Completa el asistente y reinicia si lo pide.
3. Ejecuta como administrador `Dynatest Data Collection Installer-2.4.1` y completa el asistente.
4. Reinicia el equipo y abre Dynatest Data Collection para verificar que inicia sin errores.

Los prerrequisitos (2.1.0.246) deben instalarse siempre **antes** que el instalador principal (2.4.1).
