# spatial-shell/radial-menu — Reusable radial menu widget + menubar item builder.
#
# Landed: pure-C model layer (paging, focus, thumb-extension commit) with
# synthetic-input unit tests, plus the menubar-items builder.  GTK4 + cairo
# rendering and scroll/thumb_slider wiring are deferred to the in-person
# session that designs the visual feel — those land on top of this model.

radial_menu_lib = static_library('spatial-radial-menu',
  'radial_menu.c',
  install: false,
)

radial_menu_dep = declare_dependency(
  link_with: radial_menu_lib,
  include_directories: include_directories('.'),
)

test_radial_menu = executable('test_radial_menu',
  'test_radial_menu.c',
  link_with: radial_menu_lib,
  install: false,
)

test('radial_menu',
  test_radial_menu,
  suite: 'radial_menu',
  timeout: 10,
)

menubar_items_lib = static_library('spatial-menubar-items',
  'menubar_items.c',
  install: false,
)

test_menubar_items = executable('test_menubar_items',
  'test_menubar_items.c',
  link_with: menubar_items_lib,
  include_directories: include_directories('.'),
  install: false,
)

test('radial_menubar',
  test_menubar_items,
  suite: 'radial_menubar',
  timeout: 10,
)
