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

#include "capi.h"

#include <Domain/Domain.h>
#include <Domain/Node.h>
#include <Element/Element.h>
#include <Include/whereami/whereami.h>
#include <Step/Bead.h>
#include <Toolbox/command.h>
#include <typeinfo>
#ifdef __GNUG__
#include <cxxabi.h>
#endif

extern fs::path SUANPAN_EXE;

struct sp_model {
    shared_ptr<Bead> bead = std::make_shared<Bead>();
    std::string output;
};

namespace {
    // collects everything written to the terminal and forwards complete lines to the callback
    class OutputSink final : public std::streambuf {
        std::mutex sink_mutex;
        std::string pending, captured;
        sp_log_callback callback = nullptr;
        void* user_data = nullptr;

        void forward(const std::string_view text) const {
            if(nullptr != callback) callback(text.data(), text.size(), user_data);
        }

        void append(const char* data, const std::streamsize size) {
            const std::scoped_lock lock(sink_mutex);
            captured.append(data, static_cast<size_t>(size));
            pending.append(data, static_cast<size_t>(size));
            size_t start = 0;
            for(size_t end; std::string::npos != (end = pending.find('\n', start)); start = end + 1) forward(std::string_view(pending).substr(start, end + 1 - start));
            pending.erase(0, start);
        }

    protected:
        int_type overflow(const int_type ch) override {
            if(traits_type::eq_int_type(ch, traits_type::eof())) return traits_type::not_eof(ch);
            const auto c = traits_type::to_char_type(ch);
            append(&c, 1);
            return ch;
        }

        std::streamsize xsputn(const char* data, const std::streamsize size) override {
            append(data, size);
            return size;
        }

    public:
        void set_callback(const sp_log_callback new_callback, void* new_user_data) {
            const std::scoped_lock lock(sink_mutex);
            callback = new_callback;
            user_data = new_user_data;
        }

        // flushes any incomplete line and returns all output since the last call
        std::string take() {
            const std::scoped_lock lock(sink_mutex);
            if(!pending.empty()) {
                forward(pending);
                pending.clear();
            }
            return std::exchange(captured, {});
        }
    };

    // intentionally leaked so that output emitted during static destruction still has a valid buffer
    OutputSink& sink() {
        static auto* const instance = new OutputSink;
        return *instance;
    }

    void initialise() {
        static std::once_flag flag;
        std::call_once(flag, [] {
            SUANPAN_COLOR = false;
            std::cout.rdbuf(&sink());

            if(const auto length = wai_getModulePath(nullptr, 0, nullptr); length > 0) {
                const unique_ptr<char[]> buffer(new char[length + 1]);
                wai_getModulePath(buffer.get(), length, nullptr);
                buffer[length] = '\0';
                SUANPAN_EXE = weakly_canonical(fs::path(buffer.get()));
            }
        });
    }

    template<typename F> int run(sp_model* model, F&& task) {
        if(nullptr == model) return SP_INVALID_ARGUMENT;

        initialise();
        // discard output not produced by this call
        sink().take();

        // warnings and errors are reported per call instead of being accumulated
        const auto warning_count = SUANPAN_WARNING_COUNT;
        const auto error_count = SUANPAN_ERROR_COUNT;

        auto code = SUANPAN_SUCCESS;
        try {
            code = std::forward<F>(task)();
        }
        catch(const std::bad_alloc&) {
            suanpan_error("The current platform does not have sufficient memory.\n");
            code = SP_EXCEPTION;
        }
        catch(const std::exception& e) {
            suanpan_error("Some unexpected error happens: {}.\n", e.what());
            code = SP_EXCEPTION;
        }
        catch(...) {
            suanpan_error("Some unknown error happens.\n");
            code = SP_EXCEPTION;
        }

        model->output = sink().take();

        const auto has_warning = SUANPAN_WARNING_COUNT > warning_count;
        const auto has_error = SUANPAN_ERROR_COUNT > error_count;
        SUANPAN_WARNING_COUNT = warning_count;
        SUANPAN_ERROR_COUNT = error_count;

        if(SP_EXCEPTION == code) return SP_EXCEPTION;
        if(SUANPAN_FAIL == code || has_error) return SP_ERROR;
        if(SUANPAN_EXIT == code) return SP_EXIT;
        return has_warning ? SP_WARNING : SP_OK;
    }

    const Domain* current_domain(const sp_model* model) { return dynamic_cast<const Domain*>(model->bead->get_current_domain().get()); }

    template<typename F> int query(const sp_model* model, size_t* length, F&& task) {
        if(nullptr == model || nullptr == length) return SP_INVALID_ARGUMENT;
        *length = 0;

        const auto domain = current_domain(model);
        if(nullptr == domain) return SP_NOT_FOUND;

        try {
            return std::forward<F>(task)(*domain);
        }
        catch(...) {
            return SP_EXCEPTION;
        }
    }

    template<typename T, typename C> int copy_out(const C& source, T* out, const size_t capacity, size_t* length) {
        *length = source.size();
        if(nullptr != out) {
            const auto count = std::min<size_t>(capacity, source.size());
            for(size_t I = 0; I < count; ++I) out[I] = static_cast<T>(source[I]);
        }
        return SP_OK;
    }

    template<typename T> std::vector<unsigned> sorted_tags(const Storage<T>& storage) {
        std::vector<unsigned> tags;
        tags.reserve(storage.size());
        for(auto I = storage.cbegin(); I != storage.cend(); ++I) tags.emplace_back(I->first);
        std::ranges::sort(tags);
        return tags;
    }

    // objects do not store the keyword they were created with, so the dynamic type stands in for it
    std::string class_name(const std::type_info& type) {
        std::string name = type.name();
#ifdef __GNUG__
        int status;
        if(const unique_ptr<char, decltype(&std::free)> demangled(abi::__cxa_demangle(name.c_str(), nullptr, nullptr, &status), &std::free); 0 == status) name = demangled.get();
#endif
        for(const std::string_view prefix : {"class ", "struct "})
            if(name.starts_with(prefix)) name.erase(0, prefix.size());
        return name;
    }

    int node_vector(const sp_model* model, const unsigned tag, double* out, const size_t capacity, size_t* length, const vec& (Node::*getter)() const) {
        return query(model, length, [&](const Domain& domain) {
            const auto& node = domain.get_node(tag);
            return nullptr == node ? SP_NOT_FOUND : copy_out(((*node).*getter)(), out, capacity, length);
        });
    }
} // namespace

const char* sp_version() {
    static const auto version = fmt::format("{}.{}.{}", SUANPAN_MAJOR, SUANPAN_MINOR, SUANPAN_PATCH);
    return version.c_str();
}

void sp_set_log_callback(const sp_log_callback callback, void* user_data) {
    initialise();
    sink().set_callback(callback, user_data);
}

sp_model* sp_model_new() {
    initialise();
    try {
        return new sp_model;
    }
    catch(...) {
        return nullptr;
    }
}

void sp_model_free(sp_model* model) { delete model; }

int sp_command(sp_model* model, const char* input) {
    if(nullptr == input) return SP_INVALID_ARGUMENT;

    return run(model, [&] {
        std::istringstream stream(input);
        std::string all_line, command_line;
        while(getline(stream, command_line)) {
            if(!normalise_command(all_line, command_line)) continue;
            const auto error_count = SUANPAN_ERROR_COUNT;
            const auto code = process_command(model->bead, all_line);
            if(SUANPAN_FAIL == code || SUANPAN_ERROR_COUNT > error_count) {
                suanpan_error("Stop processing at \"{}\".\n", all_line);
                return SUANPAN_FAIL;
            }
            if(SUANPAN_EXIT == code) return SUANPAN_EXIT;
            all_line.clear();
        }
        return SUANPAN_SUCCESS;
    });
}

int sp_analyze(sp_model* model) {
    return run(model, [&] { return model->bead->analyze(); });
}

const char* sp_model_output(const sp_model* model) { return nullptr == model ? "" : model->output.c_str(); }

int sp_node_tags(const sp_model* model, unsigned* out, const size_t capacity, size_t* length) {
    return query(model, length, [&](const Domain& domain) { return copy_out(sorted_tags(domain.get_node_storage()), out, capacity, length); });
}

int sp_node_coordinate(const sp_model* model, const unsigned tag, double* out, const size_t capacity, size_t* length) { return node_vector(model, tag, out, capacity, length, &Node::get_coordinate); }

int sp_node_displacement(const sp_model* model, const unsigned tag, double* out, const size_t capacity, size_t* length) { return node_vector(model, tag, out, capacity, length, &Node::get_current_displacement); }

int sp_node_resistance(const sp_model* model, const unsigned tag, double* out, const size_t capacity, size_t* length) { return node_vector(model, tag, out, capacity, length, &Node::get_current_resistance); }

int sp_node_restraints(const sp_model* model, const unsigned tag, unsigned char* out, const size_t capacity, size_t* length) {
    return query(model, length, [&](const Domain& domain) {
        const auto& node = domain.get_node(tag);
        if(nullptr == node) return SP_NOT_FOUND;
        // boundary conditions register the reordered DOFs they restrain while processing
        const auto& constrained = domain.get_constrained_dof();
        const auto& dofs = node->get_reordered_dof();
        std::vector<unsigned char> restrained(dofs.n_elem);
        for(uword I = 0; I < dofs.n_elem; ++I) restrained[I] = constrained.count(dofs(I)) > 0 ? 1 : 0;
        return copy_out(restrained, out, capacity, length);
    });
}

int sp_element_tags(const sp_model* model, unsigned* out, const size_t capacity, size_t* length) {
    return query(model, length, [&](const Domain& domain) { return copy_out(sorted_tags(domain.get_element_storage()), out, capacity, length); });
}

int sp_element_nodes(const sp_model* model, const unsigned tag, unsigned* out, const size_t capacity, size_t* length) {
    return query(model, length, [&](const Domain& domain) {
        const auto& element = domain.get_element(tag);
        return nullptr == element ? SP_NOT_FOUND : copy_out(element->get_node_encoding(), out, capacity, length);
    });
}

int sp_element_type(const sp_model* model, const unsigned tag, char* out, const size_t capacity, size_t* length) {
    return query(model, length, [&](const Domain& domain) {
        const auto& element = domain.get_element(tag);
        return nullptr == element ? SP_NOT_FOUND : copy_out(class_name(typeid(*element)), out, capacity, length);
    });
}

int sp_element_resistance(const sp_model* model, const unsigned tag, double* out, const size_t capacity, size_t* length) {
    return query(model, length, [&](const Domain& domain) {
        const auto& element = domain.get_element(tag);
        return nullptr == element ? SP_NOT_FOUND : copy_out(element->get_current_resistance(), out, capacity, length);
    });
}
