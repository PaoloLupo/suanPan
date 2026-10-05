/*******************************************************************************
 * Copyright (C) 2017-2026 Theodore Chang
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************/
/**
 * @file capi.h
 * @brief A plain C interface to drive suanPan as a shared library.
 *
 * Models are built by feeding the same commands used in `.sp` input files,
 * so every element, material, section, etc. available in the command line
 * interface is also available here. Geometry and results are read back
 * through a small set of query functions.
 *
 * All functions share global state (output streams, warning/error counters),
 * so callers must serialise calls across all models.
 *
 * Query functions follow the same pattern: they copy at most `capacity`
 * values into `out` (which may be null) and write the full length into
 * `length`, so the caller can query the size first and then fetch.
 *
 * @addtogroup Utility
 * @{
 */

#ifndef CAPI_H
#define CAPI_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32) || defined(__CYGWIN__)
#ifdef SUANPAN_CAPI_EXPORTS
#define SP_API __declspec(dllexport)
#else
#define SP_API __declspec(dllimport)
#endif
#else
#define SP_API __attribute__((visibility("default")))
#endif

/** the call succeeded without any warning */
#define SP_OK 0
/** the input requested to exit (`exit`/`quit`) */
#define SP_EXIT 1
/** the call succeeded but warnings were emitted */
#define SP_WARNING 2
/** errors were emitted or the operation failed */
#define SP_ERROR (-1)
/** an unexpected C++ exception was caught */
#define SP_EXCEPTION (-2)
/** a null pointer or otherwise invalid argument was given */
#define SP_INVALID_ARGUMENT (-3)
/** the requested object does not exist in the current domain */
#define SP_NOT_FOUND (-4)

typedef struct sp_model sp_model;

/**
 * @brief Receives terminal output, one complete line at a time.
 *
 * The callback must not call back into this API.
 */
typedef void (*sp_log_callback)(const char* text, size_t length, void* user_data);

/** @return the version string, e.g. "4.2.1" */
SP_API const char* sp_version(void);

/** @brief Sets (or clears with null) the global sink for terminal output. */
SP_API void sp_set_log_callback(sp_log_callback callback, void* user_data);

SP_API sp_model* sp_model_new(void);
SP_API void sp_model_free(sp_model* model);

/**
 * @brief Processes one or more lines of suanPan input.
 *
 * Comments, line continuations and comma delimiters are handled as in input
 * files. Processing stops at the first line that emits an error.
 */
SP_API int sp_command(sp_model* model, const char* input);

/** @brief Runs all defined steps, equivalent to the `analyze` command. */
SP_API int sp_analyze(sp_model* model);

/**
 * @return the terminal output produced by the last `sp_command` or
 * `sp_analyze` call on this model, valid until the next call on it
 */
SP_API const char* sp_model_output(const sp_model* model);

/**
 * @brief Writes the sorted tags of every object of a kind, named as in commands, such as "material",
 * "section", "constraint" or "load". Unknown kinds give SP_INVALID_ARGUMENT.
 */
SP_API int sp_tags(const sp_model* model, const char* kind, unsigned* out, size_t capacity, size_t* length);

SP_API int sp_node_tags(const sp_model* model, unsigned* out, size_t capacity, size_t* length);
SP_API int sp_node_coordinate(const sp_model* model, unsigned tag, double* out, size_t capacity, size_t* length);
SP_API int sp_node_displacement(const sp_model* model, unsigned tag, double* out, size_t capacity, size_t* length);
SP_API int sp_node_resistance(const sp_model* model, unsigned tag, double* out, size_t capacity, size_t* length);
/** @brief Writes 1 for each DOF of the node restrained by boundary conditions in the last analysis, 0 otherwise. */
SP_API int sp_node_restraints(const sp_model* model, unsigned tag, unsigned char* out, size_t capacity, size_t* length);

SP_API int sp_element_tags(const sp_model* model, unsigned* out, size_t capacity, size_t* length);
SP_API int sp_element_nodes(const sp_model* model, unsigned tag, unsigned* out, size_t capacity, size_t* length);
/** @brief Writes the class name of the element, e.g. "EB21", without a terminating NUL. */
SP_API int sp_element_type(const sp_model* model, unsigned tag, char* out, size_t capacity, size_t* length);
/**
 * @brief Writes the resisting force of the element in global coordinates, node by node and DOF by DOF,
 * which is empty until the model is analysed.
 */
SP_API int sp_element_resistance(const sp_model* model, unsigned tag, double* out, size_t capacity, size_t* length);

/**
 * @brief Writes the eigenvalues found by the last frequency analysis, the squares of the circular frequencies,
 * which is empty unless the last step was a frequency analysis.
 */
SP_API int sp_eigenvalues(const sp_model* model, double* out, size_t capacity, size_t* length);

/**
 * @brief Writes the shape of a mode, numbered from zero in the order of `sp_eigenvalues`, at the DOFs of a node.
 * Mode shapes are normalised to unit generalised mass.
 */
SP_API int sp_node_mode_shape(const sp_model* model, unsigned tag, unsigned mode, double* out, size_t capacity, size_t* length);

/**
 * @brief Writes the product of the mass matrix with the shape of a mode at the DOFs of a node.
 * Summed over nodes along a DOF, it gives the participation factor of the mode for a unit motion along that DOF.
 */
SP_API int sp_node_mode_inertia(const sp_model* model, unsigned tag, unsigned mode, double* out, size_t capacity, size_t* length);

/**
 * @brief Writes the product of the mass matrix with a unit value on the DOF `dof`, numbered from one, of every node,
 * at the DOFs of a node. Summed over nodes along the same DOF, it gives the mass that moves with a unit motion along it.
 */
SP_API int sp_node_inertia(const sp_model* model, unsigned tag, unsigned dof, double* out, size_t capacity, size_t* length);

#ifdef __cplusplus
}
#endif

#endif

//! @}
