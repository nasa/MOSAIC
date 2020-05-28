# -*- coding: utf-8 -*-
#
# Configuration file for the Sphinx documentation builder.
#
# This file does only contain a selection of the most common options. For a
# full list see the documentation:
# http://www.sphinx-doc.org/en/master/config

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys
sys.path.insert(0, os.path.abspath('.'))
sys.path.insert(0, os.path.abspath(os.path.join('..', '..')))
import sphinx_bootstrap_theme


# -- Project information -----------------------------------------------------

project = 'PDRA'
copyright = 'Copyright (c) 2019, Jet Propulsion Laboratory.'
author = 'Marc Sanchez Net'

# The version info for the project you're documenting, acts as replacement for
# |version| and |release|, also used in various other places throughout the
# built documents.
#
from pdra.core import __version__, __release__
version    = __version__   # The short X.Y version.
release    = __release__   # The full version, including alpha/beta/rc tags.
rst_epilog = '.. |version| replace:: %s' % str(version)
rst_epilog = '.. |release| replace:: %s' % str(release)


# -- General configuration ---------------------------------------------------

# If your documentation needs a minimal Sphinx version, state it here.
#
# needs_sphinx = '1.0'

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.doctest',
    'sphinx.ext.autosummary',
    'sphinx.ext.intersphinx',
    'sphinx.ext.todo',
    'sphinx.ext.coverage',
    'sphinx.ext.mathjax',
    'sphinx.ext.ifconfig',
    'sphinx.ext.viewcode',
    'sphinx.ext.githubpages',
    'recommonmark',
]

# Configuration for autosummary
autosummary_generate = True

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# The suffix(es) of source filenames.
# You can specify multiple suffix as a list of string:
#
source_suffix = ['.rst', '.md']

# The master toctree document.
master_doc = 'index'

# The language for content autogenerated by Sphinx. Refer to documentation
# for a list of supported languages.
#
# This is also used if you do content translation via gettext catalogs.
# Usually you set "language" from the command line for these cases.
language = None

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path .
exclude_patterns = ['_build', '_templates']

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = 'sphinx'


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'bootstrap'

# Theme options are theme-specific and customize the look and feel of a theme
# further.  For a list of options available for each theme, see the
# documentation.
#
html_theme_options = {
  # Render the next and previous page links in navbar. (Default: true)
  'navbar_sidebarrel': True,

  # Render the current pages TOC in the navbar. (Default: true)
  'navbar_pagenav': True,

  # Global TOC depth for "site" navbar tab. (Default: 1)
  # Switching to -1 shows all levels.
  'globaltoc_depth': 3,

  'globaltoc_includehidden': "true",

  # HTML navbar class (Default: "navbar") to attach to <div> element.
  # For black navbar, do "navbar navbar-inverse"
  'navbar_class': "navbar navbar-inverse",

  # Fix navigation bar to top of page?
  # Values: "true" (default) or "false"
  'navbar_fixed_top': "true",

  # Location of link to source.
  # Options are "nav" (default), "footer" or anything else to exclude.
  'source_link_position': "footer",

  # Bootswatch (http://bootswatch.com/) theme.
  # CHANGE THIS STYLE TO EASILY SET HOW THE DOCUMENTATION LOOKS LIKE
  #
  # Options are nothing (default) or the name of a valid theme
  # such as "amelia" or "cosmo".
  'bootswatch_theme': "spacelab",

  # Choose Bootstrap version.
  # Values: "3" (default) or "2" (in quotes)
  'bootstrap_version': "3",
}

# Add any paths that contain custom themes here, relative to this directory.
#html_theme_path = []
html_theme_path = sphinx_bootstrap_theme.get_html_theme_path()

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

# Custom sidebar templates, must be a dictionary that maps document names
# to template names.
#
# The default sidebars (for documents that don't match any pattern) are
# defined by theme itself.  Builtin themes are using these templates by
# default: ``['localtoc.html', 'relations.html', 'sourcelink.html',
# 'searchbox.html']``.
#
standard_sidebars = ['sidebartoc.html', 'sourcelink.html']
html_sidebars = {
    '*': standard_sidebars,
}


# -- Options for HTMLHelp output ---------------------------------------------

# Output file base name for HTML help builder.
htmlhelp_basename = 'DtnSimdoc'


# -- Options for LaTeX output ------------------------------------------------

latex_elements = {
    # The paper size ('letterpaper' or 'a4paper').
    #
    # 'papersize': 'letterpaper',

    # The font size ('10pt', '11pt' or '12pt').
    #
    # 'pointsize': '10pt',

    # Additional stuff for the LaTeX preamble.
    #
    # 'preamble': '',

    # Latex figure (float) alignment
    #
    # 'figure_align': 'htbp',
}

# Grouping the document tree into LaTeX files. List of tuples
# (source start file, target name, title,
#  author, documentclass [howto, manual, or own class]).
latex_documents = [
    (master_doc, 'PDRA.tex', 'PDRA Documentation',
     'Marc Sanchez Net', 'manual'),
]


# -- Options for manual page output ------------------------------------------

# One entry per manual page. List of tuples
# (source start file, name, description, authors, manual section).
man_pages = [
    (master_doc, 'pdra', 'PDRA Documentation',
     [author], 1)
]


# -- Options for Texinfo output ----------------------------------------------

# Grouping the document tree into Texinfo files. List of tuples
# (source start file, target name, title, author,
#  dir menu entry, description, category)
texinfo_documents = [
    (master_doc, 'PDRA', 'PDRA Documentation',
     author, 'PDRA', 'One line description of project.',
     'Miscellaneous'),
]


# -- Extension configuration -------------------------------------------------

# Default configuration for Sphinx autodoc
autodoc_default_options = {
    'members': None,
    'member-order': 'alphabetical',
    'special-members': '__init__',
    'show-inheritance': None,
    'private-members': None,
    'exclude-members': '__weakref__'
}

# -- Options for intersphinx extension ---------------------------------------

# Example configuration for intersphinx: refer to the Python standard library.
intersphinx_mapping = {'https://docs.python.org/': None}

# -- Options for todo extension ----------------------------------------------

# If true, `todo` and `todoList` produce output, else they produce nothing.
todo_include_todos = True