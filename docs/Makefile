# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line.
SPHINXOPTS    =
SPHINXBUILD   = sphinx-build
SPHINX2BUILD   = python2 -msphinx
SOURCEDIR     = .
BUILDDIR      = _build

PDRA_SOURCE   = ../distributed_resource_allocator/doc/source/
PDRA_BUILD    = ../distributed_resource_allocator/doc/build/

SCHED_SOURCE  = ../schedulers/docs/
SCHED_BUILD   = ../schedulers/docs/_build/


Yellow=\033[0;33m
Green=\033[0;32m
Color_Off=\033[0m

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: help Makefile

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).

pdra_gh_pages:
	@echo "$(Yellow)Building PDRA documentation$(Color_Off)"
	@$(SPHINX2BUILD) -M html "$(PDRA_SOURCE)" "$(PDRA_BUILD)" $(SPHINXOPTS) $(O)
	@echo "$(Green)PDRA documentation ready in $(PDRA_BUILD)$(Color_Off)"

schedulers_gh_pages:
	@echo "$(Yellow)Building schedulers documentation$(Color_Off)"
	@$(SPHINXBUILD) -M html "$(SCHED_SOURCE)" "$(SCHED_BUILD)" $(SPHINXOPTS) $(O)
	@echo "$(Green)Schedulers documentation ready in $(SCHED_BUILD)$(Color_Off)"

gh_pages: pdra_gh_pages schedulers_gh_pages
# First, make the cover page
	@$(SPHINXBUILD) -M html "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
# Copy the schedulers documentation output in _build/schedulers
	@cp -r $(SCHED_BUILD) $(BUILDDIR)/html/schedulers
# Copy the pdra documentation output in _build/pdra
	@cp -r $(PDRA_BUILD) $(BUILDDIR)/html/pdra
# Overwrite _build/html/schedulers.html
	@cp _schedulers.html _build/html/schedulers.html
# Overwrite _build/html/pdra.html
	@cp _pdra.html _build/html/pdra.html

	@echo "\n$(Green)The full Sphinx website is ready in $(BUILDDIR)/html/$(Color_Off)"

%: Makefile
    # First, make the cover page
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

	# Next, make the schedulers documentation

	# Next, make the PDRA documentation

	# Copy the schedulers documentation output in _build/schedulers

	# Copy the pdra documentation output in _build/pdra

	# Overwrite _build/html/schedulers.html

	# Overwrite _build/html/pdra.html